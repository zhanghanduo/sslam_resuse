/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * This file is part of sslam.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Zhang Handuo (hzhang032@e.ntu.edu.sg)
 *******************************************************/

#include "pose_graph.h"
#include <ctime>
#include <chrono>
#include <cereal/archives/binary.hpp>
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_RESET   "\x1b[0m"

using namespace std::chrono;

PoseGraph::PoseGraph():
 yaw_drift(0), load_gps_info(false), global_index(0), sequence_cnt(0),
 earliest_loop_index(-1), earliest_neighbor_index(-1), base_sequence(1),
 use_imu(false), base_initialized_(false)
{
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    posegraph_visualization->setScale(4.0);
    posegraph_visualization->setLineWidth(0.4);
//	t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    t_drift = Eigen::Vector3d(0, 0, 0);
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    gps_0_q = Eigen::Quaterniond::Identity();
    sequence_loop.push_back(false);
//    gps_cur_2_old = Eigen::Vector3d(0, 0, 0);
    prior_max_index = 0;

    first_skp = 50;
    count_ = 0;
}

PoseGraph::~PoseGraph()
{
    t_optimization.detach();
}

void PoseGraph::registerPub(ros::NodeHandle &n)
{
    pub_pg_path = n.advertise<nav_msgs::Path>("pose_graph_path", 100);
    pub_base_path = n.advertise<nav_msgs::Path>("base_path", 100);
    pub_base_points = n.advertise<sensor_msgs::PointCloud>("base_points", 100);
    pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 100);
    for (int i = 1; i < 10; i++)
        pub_path[i] = n.advertise<nav_msgs::Path>("path_" + to_string(i), 100);
}

void PoseGraph::setIMUFlag(bool _use_imu)
{
    use_imu = _use_imu;
    if(use_imu)
    {
        printf("VIO input, perform 4 DoF (x, y, z, yaw) pose graph optimization\n");
        t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    }
    else
    {
        printf("Pure VO input, perform 6 DoF pose graph optimization\n");
        t_optimization = std::thread(&PoseGraph::optimize6DoF, this);
    }

}

void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrame(std::shared_ptr<KeyFrame>& cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    if (sequence_cnt != cur_kf->sequence)
    {
        sequence_cnt++;
        sequence_loop.push_back(false);
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }
    
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
	int loop_index = -1;
    if (flag_detect_loop)
    {
//        TicToc tmp_t;
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else {
        if(DEBUG_IMAGE)
            addKeyFrameIntoImage(cur_kf);
        db.add(cur_kf->brief_descriptors);
    }
	if (loop_index != -1)
	{
//        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        std::shared_ptr<KeyFrame> old_kf = getKeyFrame(loop_index);

        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            if(old_kf->sequence == 0 && earliest_neighbor_index == -1)
                earliest_neighbor_index = prior_max_index;
            else if (prior_max_index < earliest_loop_index)
                earliest_neighbor_index = earliest_loop_index;

            Vector3d w_P_old, w_P_cur, vio_P_cur_;
            Matrix3d w_R_old, w_R_cur, vio_R_cur_;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur_, vio_R_cur_);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            if(use_imu)
            {
                shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur_).x();
                shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            }
            else
                shift_r = w_R_cur * vio_R_cur_.transpose();
            shift_t = w_P_cur - w_R_cur * vio_R_cur_.transpose() * vio_P_cur_;
            // shift vio pose of whole sequence to the world frame
//            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0 ) // && !old_kf->is_old)
//            {
//                printf("shift sequence!\n");
//                w_r_vio = shift_r;
//                w_t_vio = shift_t;
//                vio_P_cur_ = w_r_vio * vio_P_cur_ + w_t_vio;
//                vio_R_cur_ = w_r_vio *  vio_R_cur_;
//                cur_kf->updateVioPose(vio_P_cur_, vio_R_cur_);
//                auto it = keyframelist.begin();
//                for (; it != keyframelist.end(); it++)
//                {
//                    if((*it)->sequence == cur_kf->sequence)
//                    {
//                        Vector3d vio_P_cur_it;
//                        Matrix3d vio_R_cur_it;
//                        (*it)->getVioPose(vio_P_cur_it, vio_R_cur_it);
//                        vio_P_cur_it = w_r_vio * vio_P_cur_it + w_t_vio;
//                        vio_R_cur_it = w_r_vio *  vio_R_cur_it;
//                        (*it)->updateVioPose(vio_P_cur_it, vio_R_cur_it);
//                    }
//                }
//                sequence_loop[cur_kf->sequence] = true;
//            }
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
	}
	m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getVioPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    cur_kf->updatePose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    if(loop_index != -1)
        pose_stamped.pose.position.z = 0.2;
    else
        pose_stamped.pose.position.z = 0; //P.z();
//    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file(RESULT_PATH, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);
        loop_path_file.precision(6);
        loop_path_file << cur_kf->time_stamp << " ";
        loop_path_file.precision(5);
        loop_path_file  << P.x() << " "
              << P.y() << " "
              << P.z() << " "
              << Q.x() << " "
              << Q.y() << " "
              << Q.z() << " "
              << Q.w() << endl;
        loop_path_file.close();
    }
    //draw local connection
    if (SHOW_S_EDGE)
    {
        auto rit = keyframelist.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop)
        {
            //printf("has loop \n");
            std::shared_ptr<KeyFrame> connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P,P0;
            Matrix3d connected_R,R0;
            connected_KF->getPose(connected_P, connected_R);
            //cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if(cur_kf->sequence > 0)
            {
                //printf("add loop into visual \n");
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
            }
            
        }
    }
    //pose graph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);

	keyframelist.push_back(cur_kf);
    publish();
	m_keyframelist.unlock();
}

void PoseGraph::loadKeyFrame(std::shared_ptr<KeyFrame>& cur_kf, bool flag_detect_loop)
{
    cur_kf->index = global_index;
    global_index++;
    int loop_index = -1;
    if (flag_detect_loop)
       loop_index = detectLoop(cur_kf, cur_kf->index);
    else if(DEBUG_IMAGE)
        addKeyFrameIntoImage(cur_kf);

    if (loop_index != -1)
    {
        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        std::shared_ptr<KeyFrame> old_kf = getKeyFrame(loop_index);
        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }
    m_keyframelist.lock();
    if(display_base_path) {
        Vector3d P;
        Matrix3d R;
        cur_kf->getPose(P, R);
        Quaterniond Q{R};
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        base_path.poses.push_back(pose_stamped);
        base_path.header = pose_stamped.header;

        vector<cv::Point3f> points_per_frame;
        cur_kf->getPoints(points_per_frame);
        for (auto &point_ : points_per_frame) {
            geometry_msgs::Point32 p_;
            p_.x = point_.x;
            p_.y = point_.y;
            p_.z = point_.z;
            base_point_cloud.points.push_back(p_);
        }
        base_point_cloud.header = pose_stamped.header;
        base_path.poses.push_back(pose_stamped);
        base_path.header = pose_stamped.header;

        //draw local connection
        if (SHOW_S_EDGE)
        {
            auto rit = keyframelist.rbegin();
            for (int i = 0; i < 1; i++)
            {
                if (rit == keyframelist.rend())
                    break;
                Vector3d conncected_P;
                Matrix3d connected_R;
                if((*rit)->sequence == cur_kf->sequence)
                {
                    (*rit)->getPose(conncected_P, connected_R);
                    posegraph_visualization->add_edge(P, conncected_P);
                }
                rit++;
            }
        }
    }
    keyframelist.push_back(cur_kf);
    publish();
    m_keyframelist.unlock();
}

std::shared_ptr<KeyFrame> PoseGraph::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    auto it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return nullptr;
}

int PoseGraph::detectLoop(std::shared_ptr<KeyFrame>& keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->brief_descriptors);
    //printf("add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    bool find_loop = false;
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (!ret.empty())
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its neighbour
    if (!ret.empty() &&ret[0].Score > 0.05)
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {          
                find_loop = true;
                int tmp_index = ret[i].Id;
//                if (DEBUG_IMAGE && 0)
//                {
//                    auto it = image_pool.find(tmp_index);
//                    cv::Mat tmp_image = (it->second).clone();
//                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
//                    cv::hconcat(loop_result, tmp_image, loop_result);
//                }
            }

        }
/*
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
*/
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}

void PoseGraph::addKeyFrameIntoImage(std::shared_ptr<KeyFrame>& keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    int feature_num = keyframe->keypoints.size();
    cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
    putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
    image_pool[keyframe->index] = compressed_image;

}

void PoseGraph::optimize4DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            std::shared_ptr<KeyFrame> cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();

            list<std::shared_ptr<KeyFrame>>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j],
                                            t_array[i-j],
                                            euler_array[i],
                                            t_array[i]);
                  }
                }

                //add loop edge

                if((*it)->has_loop)
                {
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index],
                                                                  t_array[connected_index],
                                                                  euler_array[i],
                                                                  t_array[i]);

                }

                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            //printf("pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
}

//void PoseGraph::optimize6DoF()
//{
//    while(true)
//    {
//        int cur_index = -1;
//        int first_looped_index = -1;
//        m_optimize_buf.lock();
//        while(!optimize_buf.empty())
//        {
//            cur_index = optimize_buf.front();
//            first_looped_index = earliest_loop_index;
//            optimize_buf.pop();
//        }
//        m_optimize_buf.unlock();
//        if (cur_index != -1)
//        {
//            printf("Loop Detected \n");
//            TicToc tmp_time;
//            m_keyframelist.lock();
//            std::shared_ptr<KeyFrame> cur_kf = getKeyFrame(cur_index);
//
//            int max_length = cur_index + 1;
//
//            // w^t_i   w^q_i
//            double t_array[max_length][3];
//            double q_array[max_length][4];
//            double sequence_array[max_length];
//
//            ceres::Problem problem;
//            ceres::Solver::Options options;
//            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//            //options.minimizer_progress_to_stdout = true;
//            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
//            options.max_num_iterations = 5;
//            ceres::Solver::Summary summary;
//            ceres::LossFunction *loss_function;
//            loss_function = new ceres::HuberLoss(0.1);
//            //loss_function = new ceres::CauchyLoss(1.0);
//            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
//
//            auto it = keyframelist.begin();
//
//            int i = 0;
//            for ( ; it != keyframelist.end(); it++)
//            {
//                if ((*it)->index < first_looped_index)
//                    continue;
//                (*it)->local_index = i;
//                Quaterniond tmp_q;
//                Matrix3d tmp_r;
//                Vector3d tmp_t;
//                (*it)->getVioPose(tmp_t, tmp_r);
//                tmp_q = tmp_r;
//                t_array[i][0] = tmp_t(0);
//                t_array[i][1] = tmp_t(1);
//                t_array[i][2] = tmp_t(2);
//                q_array[i][0] = tmp_q.w();
//                q_array[i][1] = tmp_q.x();
//                q_array[i][2] = tmp_q.y();
//                q_array[i][3] = tmp_q.z();
//
//                sequence_array[i] = (*it)->sequence;
//
//                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
//                problem.AddParameterBlock(t_array[i], 3);
//
//                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
//                {
//                    problem.SetParameterBlockConstant(q_array[i]);
//                    problem.SetParameterBlockConstant(t_array[i]);
//                }
//
//                //add edge
//                for (int j = 1; j < 8; j++)
//                {
//                    if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
//                    {
//                        Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
//                        Quaterniond q_i_j = Quaterniond(q_array[i-j][0], q_array[i-j][1], q_array[i-j][2], q_array[i-j][3]);
//                        Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
//                        relative_t = q_i_j.inverse() * relative_t;
//                        Quaterniond relative_q = q_i_j.inverse() * q_i;
//                        ceres::CostFunction* vo_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
//                                                                                   relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
//                                                                                   0.1, 0.01);
//                        problem.AddResidualBlock(vo_function, nullptr, q_array[i-j], t_array[i-j], q_array[i], t_array[i]);
//                    }
//                }
//
//                //add loop edge
//
//                if((*it)->has_loop)
//                {
//                    assert((*it)->loop_index >= first_looped_index);
//                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
//                    Vector3d relative_t;
//                    relative_t = (*it)->getLoopRelativeT();
//                    Quaterniond relative_q;
//                    relative_q = (*it)->getLoopRelativeQ();
//                    ceres::CostFunction* loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
//                                                                                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
//                                                                                 0.1, 0.01);
//                    problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index], t_array[connected_index], q_array[i], t_array[i]);
//                }
//
//                if ((*it)->index == cur_index)
//                    break;
//                i++;
//            }
//            m_keyframelist.unlock();
//
//            ceres::Solve(options, &problem, &summary);
//            //std::cout << summary.BriefReport() << "\n";
//
//            //printf("pose optimization time: %f \n", tmp_time.toc());
//            /*
//            for (int j = 0 ; j < i; j++)
//            {
//                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
//            }
//            */
//            m_keyframelist.lock();
//            i = 0;
//            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
//            {
//                if ((*it)->index < first_looped_index)
//                    continue;
//                Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
//                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
//                Matrix3d tmp_r = tmp_q.toRotationMatrix();
//                (*it)-> updatePose(tmp_t, tmp_r);
//
//                if ((*it)->index == cur_index)
//                    break;
//                i++;
//            }
//
//            Vector3d cur_t, vio_t;
//            Matrix3d cur_r, vio_r;
//            cur_kf->getPose(cur_t, cur_r);
//            cur_kf->getVioPose(vio_t, vio_r);
//            m_drift.lock();
//            r_drift = cur_r * vio_r.transpose();
//            t_drift = cur_t - r_drift * vio_t;
//            m_drift.unlock();
//            //cout << "t_drift " << t_drift.transpose() << endl;
//            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
//
//            it++;
//            for (; it != keyframelist.end(); it++)
//            {
//                Vector3d P;
//                Matrix3d R;
//                (*it)->getVioPose(P, R);
//                P = r_drift * P + t_drift;
//                R = r_drift * R;
//                (*it)->updatePose(P, R);
//            }
//            m_keyframelist.unlock();
//            updatePath();
//        }
//
//        std::chrono::milliseconds dura(2000);
//        std::this_thread::sleep_for(dura);
//    }
//}


void PoseGraph::optimize6DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        int first_neighbour_index = -1;
        m_optimize_buf.lock();

        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            first_neighbour_index = earliest_neighbor_index;
            optimize_buf.pop();
        }

        m_optimize_buf.unlock();
        if (cur_index != -1 )
        {
//            printf(ANSI_COLOR_YELLOW "Loop Detected" ANSI_COLOR_RESET "\n");
            printf("Loop Detected \n");
//            printf("No: %d:\n  earliest neighbor: %d\n  earliest loop: %d\n",
//                   cur_index, first_neighbour_index, first_looped_index);
//            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            m_keyframelist.lock();
            std::shared_ptr<KeyFrame> cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            double q_array[max_length][4];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            auto it = keyframelist.begin();
            int i = 0;

            for ( ; it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_neighbour_index)
                    continue;
                if((*it)->has_loop)
                {
                    if((*it)->loop_index >= first_looped_index && (*it)->loop_index < first_neighbour_index){
//                        printf("No: %d:\n  old index: %d\n  Local index: %d\n",
//                               (*it)->index, old_kf->index, i);
                        std::shared_ptr<KeyFrame> old_kf = getKeyFrame((*it)->loop_index);
                        old_kf->local_index = i;
                        Quaterniond tmp_q;
                        Matrix3d tmp_r;
                        Vector3d tmp_t;
                        old_kf->getVioPose(tmp_t, tmp_r);
                        tmp_q = tmp_r;
                        t_array[i][0] = tmp_t(0);
                        t_array[i][1] = tmp_t(1);
                        t_array[i][2] = tmp_t(2);
                        q_array[i][0] = tmp_q.w();
                        q_array[i][1] = tmp_q.x();
                        q_array[i][2] = tmp_q.y();
                        q_array[i][3] = tmp_q.z();
                        sequence_array[i] = old_kf->sequence;

                        problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                        problem.AddParameterBlock(t_array[i], 3);

                        if (old_kf->index == first_looped_index || old_kf->sequence == 0) {
                            problem.SetParameterBlockConstant(q_array[i]);
                            problem.SetParameterBlockConstant(t_array[i]);
                        }
                        i++;
                    }
                }
            }
            int loop_i = i;

            for (it = keyframelist.begin() ; it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_neighbour_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i][0] = tmp_q.w();
                q_array[i][1] = tmp_q.x();
                q_array[i][2] = tmp_q.y();
                q_array[i][3] = tmp_q.z();
                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

//                if ((*it)->index == first_neighbour_index || (*it)->sequence == 0)
                if ((*it)->sequence == 0)
                {
                    problem.SetParameterBlockConstant(q_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add loop edge
                if((*it)->has_loop)
                {
//                    assert((*it)->loop_index >= first_looped_index);
                    std::shared_ptr<KeyFrame> old_kf = getKeyFrame((*it)->loop_index);
                    int connected_index = old_kf->local_index;
//                    printf("No: %d:\n  Local connected index: %d\n  Global connected index: %d\n",
//                            (*it)->index, connected_index, (*it)->loop_index);
//                    if(old_kf->index > first_neighbour_index)
//                        smooth = true;
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    Quaterniond relative_q;
                    relative_q = (*it)->getLoopRelativeQ();
                    ceres::CostFunction* loop_function;

                    if(old_kf->sequence == 0)
                        loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                            relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                            0.01, 0.001);
                    else
                        loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                0.1, 0.01);
                    problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index],
                            t_array[connected_index], q_array[i], t_array[i]);
                }

                //add neighborhood edge
                for (int j = loop_i + 1; j < loop_i + 8; j++)
                {
                    if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                    {
                        Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                        Quaterniond q_i_j = Quaterniond(q_array[i-j][0], q_array[i-j][1], q_array[i-j][2], q_array[i-j][3]);
                        Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                        relative_t = q_i_j.inverse() * relative_t;
                        Quaterniond relative_q = q_i_j.inverse() * q_i;
                        ceres::CostFunction* vo_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                   relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                   0.1, 0.01);
                        problem.AddResidualBlock(vo_function, nullptr, q_array[i-j], t_array[i-j], q_array[i], t_array[i]);
                    }
                }

                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            //printf("pose optimization time: %f \n", tmp_time.toc());
//            high_resolution_clock::time_point t2 = high_resolution_clock::now();
//            duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
//            opt_duration = time_span.count();
//            printf(ANSI_COLOR_RED "   opt time: %.1f ms" ANSI_COLOR_RESET "\n", opt_duration * 1000);

            m_keyframelist.lock();
            i = loop_i;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_neighbour_index)
                    continue;
                Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            r_drift = cur_r * vio_r.transpose();
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();

//            high_resolution_clock::time_point t3 = high_resolution_clock::now();
//            duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
//            update_duration = time_span2.count();
//            printf(ANSI_COLOR_RED "   update time: %.1f ms" ANSI_COLOR_RESET "\n", update_duration * 1000);

        }
        count_ ++;
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
}

void PoseGraph::updatePath()
{
    m_keyframelist.lock();
    list<std::shared_ptr<KeyFrame>>::iterator it;
    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();
    }
    if(display_base_path && !base_initialized_) {
        base_path.poses.clear();
        base_point_cloud.points.clear();
        posegraph_visualization->reset();
    }
    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file_tmp(RESULT_PATH, ios::out);
        loop_path_file_tmp.close();
    }

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
//        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time((*it)->time_stamp);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();

        if((*it)->sequence == 0 && display_base_path && !base_initialized_)
        {
            vector<cv::Point3f> points_per_frame;
            (*it)->getPoints(points_per_frame);
            for (auto &point_ : points_per_frame) {
                geometry_msgs::Point32 p_;
                p_.x = point_.x;
                p_.y = point_.y;
                p_.z = point_.z;
                base_point_cloud.points.push_back(p_);
            }
            base_point_cloud.header = pose_stamped.header;
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        else if((*it)->sequence != 0)
        {
            if((*it)->has_loop)
                pose_stamped.pose.position.z = 0.2;
            else
                pose_stamped.pose.position.z = 0;
            path[(*it)->sequence].poses.push_back(pose_stamped);
            path[(*it)->sequence].header = pose_stamped.header;
        }

        if (SAVE_LOOP_PATH && (*it)->sequence != 0)
        {
            ofstream loop_path_file(RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(6);
            loop_path_file << (*it)->time_stamp << " ";
            loop_path_file.precision(5);
            loop_path_file  << P.x() << " "
                  << P.y() << " "
                  << P.z() << " "
                  << Q.x() << " "
                  << Q.y() << " "
                  << Q.z() << " "
                  << Q.w() << endl;
            loop_path_file.close();
        }
        //draw local connection
        if (SHOW_S_EDGE)
        {
            auto rit = keyframelist.rbegin();
            list<std::shared_ptr<KeyFrame>>::reverse_iterator lrit;
            for (; rit != keyframelist.rend(); rit++)  
            {  
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())
                            break;
                        if((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            (*lrit)->getPose(conncected_P, connected_R);
                            posegraph_visualization->add_edge(P, conncected_P);
                        }
                        lrit++;
                    }
                    break;
                }
            } 
        }
        if (SHOW_L_EDGE)
        {
            if ((*it)->has_loop && (*it)->sequence == sequence_cnt)
            {

                std::shared_ptr<KeyFrame> connected_KF = getKeyFrame((*it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);
                //(*it)->getVioPose(P, R);
                (*it)->getPose(P, R);
                if((*it)->sequence > 0)
                {
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
                }
            }
        }

    }
    if(!base_initialized_)
        base_initialized_ = true;
    publish();
    m_keyframelist.unlock();
}

void PoseGraph::savePoseGraph() {
    m_keyframelist.lock();
    TicToc tmp_t;
    printf("pose graph path: %s\n", POSE_GRAPH_SAVE_PATH.c_str());
    printf("pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "/" + POSE_GRAPH_SAVE_NAME;

    std::ofstream out(file_path, std::ios_base::binary);
    if (!out) {
        std::cerr << "Cannot Write to Pose Graph Map: " << file_path << std::endl;
        exit(-1);
    }

//    Eigen::Matrix3d rot_oldcam0_2_enu = gps_0_q * rot_cam2imu;
//    Eigen::Vector3d t_oldcam0_2_enu = gps_0_trans;

    auto it = keyframelist.begin();
    for(; it != keyframelist.end(); it++)
    {
//        Eigen::Matrix3d rot_oldcami_2_enu = (*it)->R_w_i;
        Eigen::Vector3d t_oldcami_2_enu = (*it)->T_w_i + gps_0_trans;
//        (*it)->updateEnuPose(t_oldcami_2_enu, rot_oldcami_2_enu);
        (*it)->updateEnuPosition(t_oldcami_2_enu);
    }

    cereal::BinaryOutputArchive oa(out);
    oa(CEREAL_NVP(keyframelist), CEREAL_NVP(db), CEREAL_NVP(gps_0_trans), CEREAL_NVP(gps_0_q));
    std::cout << " ... done" << std::endl;
    out.close();

    if (DEBUG_IMAGE)
    {
        list<std::shared_ptr<KeyFrame>>::iterator it;
        for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
            std::string image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);
        }
    }

    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}

void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;
    string file_path = POSE_GRAPH_SAVE_PATH + "/" + POSE_GRAPH_SAVE_NAME;
    printf("load pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");

    std::ifstream in(file_path, std::ios_base::binary);
    if (!in) {
        std::cerr << "Cannot Open Pose Graph Map: " << file_path << " , Create a new one" << std::endl;
        return;
    }

    cereal::BinaryInputArchive ia(in);
    std::list<std::shared_ptr<KeyFrame>> tmp_keyframe_list;
    Vector3d gps_old_trans;
    Quaterniond gps_old_q;

    ia( CEREAL_NVP(tmp_keyframe_list), CEREAL_NVP(db), CEREAL_NVP(gps_old_trans), CEREAL_NVP(gps_old_q) );

    Matrix3d R_enu_2curgps0, R_old_2_cur;
    Vector3d t_enu_2curgps0, t_old_2_cur;

    if(load_gps_info) {
        R_enu_2curgps0 = gps_0_q.inverse().toRotationMatrix();
        t_enu_2curgps0 = - R_enu_2curgps0 * gps_0_trans;
        R_old_2_cur = R_enu_2curgps0 * gps_old_q;
        t_old_2_cur = R_enu_2curgps0 * gps_old_trans + t_enu_2curgps0;
    }

//    int cnt = 0;
    for(auto& keyframe_ : tmp_keyframe_list)
    {
        cv::Mat img_;
        if (DEBUG_IMAGE) {
            std::string image_path;
            int index_ = keyframe_->index;
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index_) + "_image.png";
            img_ = cv::imread(image_path.c_str(), 0);
            keyframe_->image = img_;
        }

        if(load_gps_info) {
            Eigen::Matrix3d R_oldimuk_2curimu0;
            Eigen::Vector3d t_oldimuk_2curimu0;
            R_oldimuk_2curimu0 = R_enu_2curgps0 * keyframe_->R_w_i;
            t_oldimuk_2curimu0 = R_enu_2curgps0 * keyframe_->T_enu_i + t_enu_2curgps0;
            keyframe_->updateVioPose(t_oldimuk_2curimu0, R_oldimuk_2curimu0);
            keyframe_->updatePoints(t_old_2_cur, R_old_2_cur);
            keyframe_->reset();
        }

        loadKeyFrame(keyframe_, false);

//        if (cnt % 20 == 0)
//            publish();
//        cnt++;
    }
    prior_max_index = global_index;
    if(!load_gps_info)
        printf("GPS information time out (20 seconds), use local information instead.\n");

    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
//    base_sequence = 0;
}

void PoseGraph::publish()
{
    for (int i = 1; i <= sequence_cnt; i++)
    {
        pub_path[i].publish(path[i]);
    }
    pub_pg_path.publish(path[1]);
    posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
    if(display_base_path) {
        pub_base_path.publish(base_path);
        pub_base_points.publish(base_point_cloud);
    }
    //pose graph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}

#pragma clang diagnostic pop