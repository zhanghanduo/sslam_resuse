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

#include "gps_graph.h"
#include <ctime>
#include <chrono>
#include <cereal/archives/binary.hpp>

using namespace std::chrono;

namespace gps_graph {

    GPSGraph::GPSGraph() :
		    yaw_drift(0), load_gps_info(false), global_index(0),
		    sequence_cnt(0), gps_bad(false), initGPS(false), newGPS(false){
        WGPS_T_WVIO = Eigen::Matrix4d::Identity();
        gpsgraph_visualization = new pose_graph::CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
        gpsgraph_visualization->setScale(4.0);
        gpsgraph_visualization->setLineWidth(0.4);
//	t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        gps_0_q = Eigen::Quaterniond::Identity();
        sequence_loop.push_back(false);
    }

    GPSGraph::~GPSGraph() {
        t_optimization.detach();
    }

    bool GPSGraph::GPSAvailable(double t) {

        return !gpsBuf.empty() && t <= gpsBuf.back().first;
    }

    void GPSGraph::inputGPS_xyz(double t, double x, double y, double z, double posAccuracy_x, double posAccuracy_y)
    {
        gps_bad = posAccuracy_x > 0.007 || posAccuracy_y > 0.007;
        if(!gps_bad) {
            double xyz[2];
            if (!initGPS) {
                offset.x() = x;
                offset.y() = y;
                xyz[0] = 0;
                xyz[1] = 0;
                initGPS = true;
            } else {
                xyz[0] = x - offset.x();
                xyz[1] = y - offset.y();
            }
            Vector5d gpsPos;
            gpsPos << xyz[0], xyz[1], z, posAccuracy_x, posAccuracy_y;
            m_buf.lock();
            gpsBuf.push(make_pair(t, gpsPos));
            m_buf.unlock();
            newGPS = true;
        } else
            cout << "gps bad! cov x: " << posAccuracy_x << " y: " << posAccuracy_y << endl;
    }

    void GPSGraph::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
    {
        mPoseMap.lock();
        vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(),
                                 OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
        localPoseMap[t] = localPose;


        Eigen::Quaterniond globalQ;
        globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
        Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
        vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                                  globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
        globalPoseMap[t] = globalPose;
        lastP = globalP;
        lastQ = globalQ;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(t);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = lastP.x();
        pose_stamped.pose.position.y = lastP.y();
        pose_stamped.pose.position.z = lastP.z();
        pose_stamped.pose.orientation.x = lastQ.x();
        pose_stamped.pose.orientation.y = lastQ.y();
        pose_stamped.pose.orientation.z = lastQ.z();
        pose_stamped.pose.orientation.w = lastQ.w();
        global_path.header = pose_stamped.header;
        global_path.poses.push_back(pose_stamped);

        mPoseMap.unlock();
    }


    void GPSGraph::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
    {
        odomP = lastP;
        odomQ = lastQ;
    }

    void GPSGraph::registerPub(ros::NodeHandle &n) {
        pub_pg_path = n.advertise<nav_msgs::Path>("gps_graph_path", 100);
        pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("gps_graph", 100);
    }

    void GPSGraph::startOptimization() {
            t_optimization = std::thread(&GPSGraph::optimize6DoF, this);
    }

    void GPSGraph::loadVocabulary(const std::string& voc_path) {
        voc = new BriefVocabulary(voc_path);
        db.setVocabulary(*voc, false, 0);
    }

    void GPSGraph::addKeyFrame(std::shared_ptr<pose_graph::KeyFrame> &cur_kf, bool flag_gps) {
        //shift to base frame
//        cout << "add keyframe" << endl;
        Vector3d vio_P_cur;
        Matrix3d vio_R_cur;
        if (sequence_cnt != cur_kf->sequence) {
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
        vio_R_cur = w_r_vio * vio_R_cur;
        cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
        cur_kf->index = global_index;
        global_index++;

        // TODO: try to put db add into flag_gps.
        db.add(cur_kf->brief_descriptors);

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

        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        path[sequence_cnt].poses.push_back(pose_stamped);
        path[sequence_cnt].header = pose_stamped.header;

        if (SAVE_LOOP_PATH) {
            ofstream loop_path_file(RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(6);
            loop_path_file << cur_kf->time_stamp << " ";
            loop_path_file.precision(5);
            loop_path_file << P.x() << " "
                           << P.y() << " "
                           << P.z() << " "
                           << Q.x() << " "
                           << Q.y() << " "
                           << Q.z() << " "
                           << Q.w() << endl;
            loop_path_file.close();
        }
        //pose graph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);

        keyframelist.push_back(cur_kf);
        publish();
        m_keyframelist.unlock();
        if(flag_gps) {
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }

    std::shared_ptr<pose_graph::KeyFrame> GPSGraph::getKeyFrame(int index) {
//    unique_lock<mutex> lock(m_keyframelist);
        auto it = keyframelist.begin();
        for (; it != keyframelist.end(); it++) {
            if ((*it)->index == index)
                break;
        }
        if (it != keyframelist.end())
            return *it;
        else
            return nullptr;
    }

    void GPSGraph::addKeyFrameIntoImage(std::shared_ptr<pose_graph::KeyFrame> &keyframe) {
        // put image into image_pool; for visualization
        cv::Mat compressed_image;
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX,
                0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;

    }

    void GPSGraph::optimize6DoF() {
        while (true) {
            int cur_index = -1;
            m_optimize_buf.lock();
            while (!optimize_buf.empty()) {
                cur_index = optimize_buf.front();
                optimize_buf.pop();
            }
            m_optimize_buf.unlock();
            if(cur_index != -1)
            {
                m_keyframelist.lock();
//                newGPS = false;
                // 1. Construct ceres problem
                std::shared_ptr<pose_graph::KeyFrame> cur_kf = getKeyFrame(cur_index);
                const int max_length = cur_index + 1;
                double t_array[max_length][3];
                double q_array[max_length][4];

                ceres::Problem problem;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
                //options.minimizer_progress_to_stdout = true;
                //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
                options.max_num_iterations = 6;
                ceres::Solver::Summary summary;
                ceres::LossFunction *loss_function;
                loss_function = new ceres::HuberLoss(1.0);
                ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

                // 2. Add parameter blocks
                auto it = keyframelist.begin();
                int i = 0;
                int bound = max(0, cur_index - 100);
                for (; it != keyframelist.end(); it++)
                {
                    if ((*it)->index < bound)
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
                    problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                    problem.AddParameterBlock(t_array[i], 3);

                    //Adjacent factor
                    for (int j = 1; j < 3; j++)
                    {
                        if (i - j >= 0)
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
                    //GPS factor
                    if((*it)->has_gps)
                    {
//                        cout << "x: " << (*it)->gps_info[0] << "  y: " << (*it)->gps_info[1] << "  z: " << (*it)->gps_info[2] << endl
//                        << " cov: " << (*it)->gps_info[3] << endl;
                        ceres::CostFunction* gps_function = TError::Create((*it)->gps_info[0], (*it)->gps_info[1],
                                                                           (*it)->gps_info[2], (*it)->gps_info[3], (*it)->gps_info[4]);
                        problem.AddResidualBlock(gps_function, loss_function, t_array[i]);
                    }
                    if ((*it)->index == cur_index)
                        break;
                    i++;
                }
                m_keyframelist.unlock();
                // 3. Solve the problem
                ceres::Solve(options, &problem, &summary);
//                std::cout << summary.BriefReport() << "\n";

                // 4. Update global path
                m_keyframelist.lock();
                i = 0;
                for (it = keyframelist.begin(); it != keyframelist.end(); it++)
                {
                    if ((*it)->index < bound)
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

                it ++;
                for (; it != keyframelist.end(); it++) {
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

    void GPSGraph::updatePath() {
        m_keyframelist.lock();
        list<std::shared_ptr<pose_graph::KeyFrame>>::iterator it;
        for (int i = 1; i <= sequence_cnt; i++) {
            path[i].poses.clear();
        }
        if (SAVE_LOOP_PATH) {
            ofstream loop_path_file_tmp(RESULT_PATH, ios::out);
            loop_path_file_tmp.close();
        }

        for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
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

            if ((*it)->sequence != 0) {
//            if((*it)->has_loop)
//                pose_stamped.pose.position.z = 0.2;
//            else
//                pose_stamped.pose.position.z = 0;
                path[(*it)->sequence].poses.push_back(pose_stamped);
                path[(*it)->sequence].header = pose_stamped.header;
            }

            if (SAVE_LOOP_PATH && (*it)->sequence != 0) {
                ofstream loop_path_file(RESULT_PATH, ios::app);
                loop_path_file.setf(ios::fixed, ios::floatfield);
                loop_path_file.precision(6);
                loop_path_file << (*it)->time_stamp << " ";
                loop_path_file.precision(5);
                loop_path_file << P.x() << " "
                               << P.y() << " "
                               << P.z() << " "
                               << Q.x() << " "
                               << Q.y() << " "
                               << Q.z() << " "
                               << Q.w() << endl;
                loop_path_file.close();
            }

        }

        publish();
        m_keyframelist.unlock();
    }

    void GPSGraph::savePoseGraph() {
        m_keyframelist.lock();
        pose_graph::TicToc tmp_t;
        printf("pose graph path: %s\n", POSE_GRAPH_SAVE_PATH.c_str());
        printf("pose graph saving... \n");
        string file_path = POSE_GRAPH_SAVE_PATH + "/" + POSE_GRAPH_SAVE_NAME;

        std::ofstream out(file_path, std::ios_base::binary);
        if (!out) {
            std::cerr << "Cannot Write to Pose Graph Map: " << file_path << std::endl;
            exit(-1);
        }

        // TODO: No need to keep all keyframes. We can select every 10 keyframes to make map compact and
        // map matching process simpler.
//        copy_every_n(keyframelist.begin(), keyframelist.end(), short_keyframes.begin(), 2);

        auto it = keyframelist.begin();
        for (; it != keyframelist.end(); it++) {
//        Eigen::Matrix3d rot_oldcami_2_enu = (*it)->R_w_i;
            Eigen::Vector3d t_oldcami_2_enu = (*it)->T_w_i + gps_0_trans;
//        (*it)->updateEnuPose(t_oldcami_2_enu, rot_oldcami_2_enu);
            (*it)->updateEnuPosition(t_oldcami_2_enu);
        }

        cereal::BinaryOutputArchive oa(out);
        oa(CEREAL_NVP(keyframelist), CEREAL_NVP(gps_0_trans), CEREAL_NVP(gps_0_q), CEREAL_NVP(db));
        std::cout << " ... done" << std::endl;
        out.close();

        if (DEBUG_IMAGE) {
            list<std::shared_ptr<pose_graph::KeyFrame>>::iterator it_im;
            for (it_im = keyframelist.begin(); it_im != keyframelist.end(); it_im++) {
                std::string image_path = POSE_GRAPH_SAVE_PATH + "/" + to_string((*it_im)->index) + "_image.png";
                imwrite(image_path.c_str(), (*it_im)->image);
            }
        }

        printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
        m_keyframelist.unlock();
    }

    void GPSGraph::loadPoseGraph() {
        string file_path = POSE_GRAPH_SAVE_PATH + "/" + POSE_GRAPH_SAVE_NAME;
        printf("load pose graph from: %s \n", file_path.c_str());
        printf("pose graph loading...\n");

        std::ifstream in(file_path, std::ios_base::binary);
        if (!in) {
            std::cerr << "Cannot Open Pose Graph Map: " << file_path << " , Create a new one" << std::endl;
            return;
        }

        cereal::BinaryInputArchive ia(in);
        std::list<std::shared_ptr<pose_graph::KeyFrame>> tmp_keyframe_list;
        Vector3d gps_old_trans;
        Quaterniond gps_old_q;

        ia(CEREAL_NVP(tmp_keyframe_list), CEREAL_NVP(gps_old_trans), CEREAL_NVP(gps_old_q), CEREAL_NVP(db));

        Matrix3d R_old_2_cur;
        Vector3d t_old_2_cur;

        if (load_gps_info) {
//        R_enu_2curgps0 = gps_0_q.inverse().toRotationMatrix();
//        t_enu_2curgps0 = - R_enu_2curgps0 * gps_0_trans;
            t_old_2_cur = gps_old_trans - gps_0_trans;
        }

//    int cnt = 0;
        for (auto &keyframe_ : tmp_keyframe_list) {
            cv::Mat img_;
            if (DEBUG_IMAGE) {
                std::string image_path;
                int index_ = keyframe_->index;
                image_path = POSE_GRAPH_SAVE_PATH + "/" + to_string(index_) + "_image.png";
                img_ = cv::imread(image_path.c_str(), 0);
                keyframe_->image = img_;
            }

            if (load_gps_info) {
                Eigen::Matrix3d R_oldimuk_2curimu0;
                Eigen::Vector3d t_oldimuk_2curimu0;
//            R_oldimuk_2curimu0 = keyframe_->R_w_i;
                t_oldimuk_2curimu0 = keyframe_->T_enu_i - gps_0_trans;
//            keyframe_->updateVioPose(t_oldimuk_2curimu0, R_oldimuk_2curimu0);
//            keyframe_->updatePoints(t_old_2_cur, R_old_2_cur);
                keyframe_->updateVioPose_norot(t_oldimuk_2curimu0);
                keyframe_->updatePoints_norot(t_old_2_cur);
                keyframe_->reset();
            }

//            loadKeyFrame(keyframe_, false);

//        if (cnt % 20 == 0)
//            publish();
//        cnt++;
        }
        if (!load_gps_info)
            printf("GPS information time out (20 seconds), use local information instead.\n");

//    base_sequence = 0;
    }

    void GPSGraph::publish() {
//        for (int i = 1; i <= sequence_cnt; i++) {
//            pub_path[i].publish(path[i]);
//        }
        pub_pg_path.publish(path[1]);
        gpsgraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
        //pose graph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
    }

}
