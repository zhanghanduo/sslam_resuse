/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file feature_manager.cpp
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief SSLAM-estimator feature database.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#include "feature_manager.h"
#ifdef SHOW_PROFILING
	#include "../utility/log/Profiler.hpp"
	#include "../utility/log/Logger.hpp"
#endif

/**
 * @namespace slam_estimator
 */
namespace slam_estimator {

    int FeaturePerId::endFrame() {
        return start_frame + feature_per_frame.size() - 1;
    }

    FeatureManager::FeatureManager(Matrix3d _Rs[])
            : Rs(_Rs) {
        for (int i = 0; i < NUM_OF_CAM; i++)
            ric[i].setIdentity();
    }

    void FeatureManager::setRic(Matrix3d _ric[]) {
        for (int i = 0; i < NUM_OF_CAM; i++) {
            ric[i] = _ric[i];
        }
    }

    void FeatureManager::clearState() {
        feature.clear();
    }

    int FeatureManager::getFeatureCount() {
        int cnt = 0;
        for (auto &it : feature) {
            it.used_num = it.feature_per_frame.size();
            if (it.used_num >= 4) {
                cnt++;
            }
        }
        return cnt;
    }

    bool FeatureManager::addFeatureCheckParallax(int frame_count,
                                                 const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                                                 double td) {
#ifdef SHOW_PROFILING
	    Logger::Write(printFullPrecision( utility::Timer::now() ) + "            [Feature Manager] Input features: " +
	    std::to_string(image.size()) + "\n");
	    Logger::Write(printFullPrecision( utility::Timer::now() ) + "            [Feature Manager] No. of stable features: " +
	    std::to_string(getFeatureCount()) + "\n");
#endif // SHOW_PROFILING

        double parallax_sum = 0;
        int parallax_num = 0;
        last_track_num = 0;
//        last_average_parallax = 0;
        new_feature_num = 0;
        long_track_num = 0;
        for (auto &id_pts : image) {
        	// 1. Feature properties of one point in a frame.
            FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
            assert(id_pts.second[0].first == 0);
            // Check whether the feature has both left and right observations.
            // If so, add right image observation to FeaturePerFrame.
            if (id_pts.second.size() == 2) {
                f_per_fra.rightObservation(id_pts.second[1].second);
                assert(id_pts.second[1].first == 1);
            }

            int feature_id = id_pts.first;
            // Lambda expression to find the index of FeaturePerId (map point) among
	        // all features of map point database --- std::list "feature"
            auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it) {
                return it.feature_id == feature_id;
            });

            if (it == feature.end()) {
            	// If cannot find previous observation of this feature, push it into the current database,
            	// where its start_frame is current frame_count.
                feature.emplace_back(feature_id, frame_count);
                feature.back().feature_per_frame.push_back(f_per_fra);
                new_feature_num++;
            } else if (it->feature_id == feature_id) {
                it->feature_per_frame.push_back(f_per_fra);
                last_track_num++;
                // For feature consistently tracking above 4 times, we define it long_track.
                if (it->feature_per_frame.size() >= 4)
                    long_track_num++;
            }
        }

        if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
            return true;

        for (auto &it_per_id : feature) {
        	// For features that have been tracked for more than 3 times
        	// and still being tracked.
            if (it_per_id.start_frame <= frame_count - 2 &&
                it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) >= frame_count) {
                parallax_sum += compensatedParallax2(it_per_id, frame_count);
                parallax_num++;
            }
        }

        if (parallax_num == 0)
            return true;
        else {
#ifdef SHOW_PROFILING
	        Logger::Write("             [Feature Manager] parallax sum value: " + std::to_string(parallax_sum) +
	        " ,   parallax number: " + std::to_string(parallax_num) + "\n");
	        WriteToLog("             [Feature Manager] current parallax: ", parallax_sum / parallax_num * FOCAL_LENGTH);
#endif // SHOW_PROFILING
			// If last average parallax is above MIN_PARALLAX (*FOCAL_LENGTH) pixels, we decide it is keyframe.
            return parallax_sum / parallax_num >= MIN_PARALLAX;
        }
    }

    vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r) {
        vector<pair<Vector3d, Vector3d>> corres;
        for (auto &it : feature) {
            if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r) {
                Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
                int idx_l = frame_count_l - it.start_frame;
                int idx_r = frame_count_r - it.start_frame;

                a = it.feature_per_frame[idx_l].point;

                b = it.feature_per_frame[idx_r].point;

                corres.emplace_back(a, b);
            }
        }
        return corres;
    }

    void FeatureManager::setDepth(const VectorXd &x) {
        int feature_index = -1;
        for (auto &it_per_id : feature) {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;

            it_per_id.estimated_depth = 1.0 / x(++feature_index);
            //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
            if (it_per_id.estimated_depth < 0) {
                it_per_id.solve_flag = 2;
            } else
                it_per_id.solve_flag = 1;
        }
    }

    void FeatureManager::removeFailures() {
        for (auto it = feature.begin(), it_next = feature.begin();
             it != feature.end(); it = it_next) {
            it_next++;
            if (it->solve_flag == 2)
                feature.erase(it);
        }
    }

    void FeatureManager::clearDepth() {
        for (auto &it_per_id : feature)
            it_per_id.estimated_depth = -1;
    }

    VectorXd FeatureManager::getDepthVector() {
        VectorXd dep_vec(getFeatureCount());
        int feature_index = -1;
        for (auto &it_per_id : feature) {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;
            dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
//        dep_vec(++feature_index) = it_per_id->estimated_depth;
        }
        return dep_vec;
    }


    // For the theory of triangulation using SVD decomposition, kindly refer to:
    // http://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf
    void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                          Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d) {
        Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
        design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
        design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
        design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
        design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
        Eigen::Vector4d triangulated_point;
        triangulated_point =
                design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
        point_3d(0) = triangulated_point(0) / triangulated_point(3);
        point_3d(1) = triangulated_point(1) / triangulated_point(3);
        point_3d(2) = triangulated_point(2) / triangulated_point(3);
    }


    bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P,
                                        vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D) {
        Eigen::Matrix3d R_initial;
        Eigen::Vector3d P_initial;

        // w_T_cam ---> cam_T_w
        R_initial = R.inverse();
        P_initial = -(R_initial * P);

//    printf("pnp size %d \n",(int)pts2D_.size() );
        if (int(pts2D.size()) < 4) {
            printf("feature tracking not enough, please slowly move you device! \n");
            return false;
        }

        cv::Mat r, rvec, t, D, tmp_r;
        cv::eigen2cv(R_initial, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_initial, t);
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        bool pnp_succ;
        pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, true);
        //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

        if (!pnp_succ) {
            printf("pnp failed ! \n");
            return false;
        }
        cv::Rodrigues(rvec, r);
//    cout << "r " << endl << r << endl;
        Eigen::MatrixXd R_pnp;
        cv::cv2eigen(r, R_pnp);
        Eigen::MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);

        // cam_T_w ---> w_T_cam
        R = R_pnp.transpose();
        // cout << "R: " << endl << R << endl;
        Eigen::Matrix3d R_ = R;
        Eigen::Vector3d P_ = R_ * (-T_pnp);
        // cout << "P: " << endl << P_ << endl;
//    P = R * (-T_pnp);
        P = P_;
        return true;
    }

    void
    FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps_[], Matrix3d Rs_[], Vector3d tic_[], Matrix3d ric_[]) {
        if (frameCnt > 0) {
            vector<cv::Point2f> pts2D;
            vector<cv::Point3f> pts3D;
            for (auto &it_per_id : feature) {
                if (it_per_id.estimated_depth > 0) {
//                ROS_WARN("Enter feature point depth > 0 condition ---");
                    int index = frameCnt - it_per_id.start_frame;
                    if ((int) it_per_id.feature_per_frame.size() >= index + 1) {
//                    ROS_WARN("Enter feature num > index condition ---");
                        Vector3d ptsInCam =
                                ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic_[0];
                        Vector3d ptsInWorld = Rs_[it_per_id.start_frame] * ptsInCam + Ps_[it_per_id.start_frame];

                        cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                        cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(),
                                            it_per_id.feature_per_frame[index].point.y());
                        pts3D.push_back(point3d);
                        pts2D.push_back(point2d);
                    }
                }
            }
            Eigen::Matrix3d RCam;
            Eigen::Vector3d PCam;
            // trans to w_T_cam
            RCam = Rs_[frameCnt - 1] * ric[0];
            PCam = Rs_[frameCnt - 1] * tic_[0] + Ps_[frameCnt - 1];
//        ROS_WARN("initFramePoseByPnP frameCnt %d, feature num %d", frameCnt, feature.size());

            if (solvePoseByPnP(RCam, PCam, pts2D, pts3D)) {
                // trans to w_T_imu
                Rs_[frameCnt] = RCam * ric[0].transpose();
                Ps_[frameCnt] = -RCam * ric[0].transpose() * tic_[0] + PCam;

                Eigen::Quaterniond Q(Rs_[frameCnt]);
//            cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
//            cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
            } else
                ROS_WARN("solvePnP unsuccessful! ---");
        }
    }

    void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]) {
        for (auto &it_per_id : feature) {
            if (it_per_id.estimated_depth > 0)
                continue;

            if (STEREO && it_per_id.feature_per_frame[0].is_stereo) {
                int imu_i = it_per_id.start_frame;
                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
                Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;
                //cout << "left pose " << leftPose << endl;

                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
                Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;
                //cout << "right pose " << rightPose << endl;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.feature_per_frame[0].point.head(2);
                point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
                //cout << "point0 " << point0.transpose() << endl;
                //cout << "point1 " << point1.transpose() << endl;

                triangulatePoint(leftPose, rightPose, point0, point1, point3d);
                Eigen::Vector3d localPoint;
                localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
                double depth = localPoint.z();
                if (depth > 0)
                    it_per_id.estimated_depth = depth;
                else
                    it_per_id.estimated_depth = INIT_DEPTH;
                /*
                Vector3d ptsGt = pts_gt[it_per_id.feature_id];
                printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                                ptsGt.x(), ptsGt.y(), ptsGt.z());
                */
                continue;
            } else if (it_per_id.feature_per_frame.size() > 1) {
                int imu_i = it_per_id.start_frame;
                Eigen::Matrix<double, 3, 4> leftPose;
                Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
                Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
                leftPose.leftCols<3>() = R0.transpose();
                leftPose.rightCols<1>() = -R0.transpose() * t0;

                imu_i++;
                Eigen::Matrix<double, 3, 4> rightPose;
                Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
                Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
                rightPose.leftCols<3>() = R1.transpose();
                rightPose.rightCols<1>() = -R1.transpose() * t1;

                Eigen::Vector2d point0, point1;
                Eigen::Vector3d point3d;
                point0 = it_per_id.feature_per_frame[0].point.head(2);
                point1 = it_per_id.feature_per_frame[1].point.head(2);
                triangulatePoint(leftPose, rightPose, point0, point1, point3d);
                Eigen::Vector3d localPoint;
                localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
                double depth = localPoint.z();
                if (depth > 0)
                    it_per_id.estimated_depth = depth;
                else
                    it_per_id.estimated_depth = INIT_DEPTH;
                /*
                Vector3d ptsGt = pts_gt[it_per_id.feature_id];
                printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                                ptsGt.x(), ptsGt.y(), ptsGt.z());
                */
                continue;
            }
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

            Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
            int svd_idx = 0;

            Eigen::Matrix<double, 3, 4> P0;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            P0.leftCols<3>() = Eigen::Matrix3d::Identity();
            P0.rightCols<1>() = Eigen::Vector3d::Zero();

            for (auto &it_per_frame : it_per_id.feature_per_frame) {
                imu_j++;

                Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
                Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
                Eigen::Vector3d t = R0.transpose() * (t1 - t0);
                Eigen::Matrix3d R = R0.transpose() * R1;
                Eigen::Matrix<double, 3, 4> P;
                P.leftCols<3>() = R.transpose();
                P.rightCols<1>() = -R.transpose() * t;
                Eigen::Vector3d f = it_per_frame.point.normalized();
                svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
                svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

                if (imu_i == imu_j)
                    continue;
            }
//        ROS_ASSERT(svd_idx == svd_A.rows());
            Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A,
                                                                      Eigen::ComputeThinV).matrixV().rightCols<1>();
            double svd_result = svd_V[2] / svd_V[3];
            //it_per_id->estimated_depth = -b / A;
            //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

            it_per_id.estimated_depth = svd_result;

            if (it_per_id.estimated_depth < 0.1) {
                it_per_id.estimated_depth = INIT_DEPTH;
            }

        }
    }

    void FeatureManager::removeOutlier(set<int> &outlierIndex) {
        std::set<int>::iterator itSet;
        for (auto it = feature.begin(), it_next = feature.begin();
             it != feature.end(); it = it_next) {
            it_next++;
            int index = it->feature_id;
            itSet = outlierIndex.find(index);
            if (itSet != outlierIndex.end()) {
                feature.erase(it);
                //printf("remove outlier %d \n", index);
            }
        }
    }

    void FeatureManager::removeBackShiftDepth(const Eigen::Matrix3d &marg_R, const Eigen::Vector3d& marg_P,
                                              const Eigen::Matrix3d& new_R, const Eigen::Vector3d new_P) {
        for (auto it = feature.begin(), it_next = feature.begin();
             it != feature.end(); it = it_next) {
            it_next++;

            if (it->start_frame != 0)
                it->start_frame--;
            else {
                Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
                it->feature_per_frame.erase(it->feature_per_frame.begin());
                if (it->feature_per_frame.size() < 2) {
                    feature.erase(it);
                    continue;
                } else {
                    Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                    Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                    Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                    double dep_j = pts_j(2);
                    if (dep_j > 0)
                        it->estimated_depth = dep_j;
                    else
                        it->estimated_depth = INIT_DEPTH;
                }
            }
            // remove tracking-lost feature after marginalize
            /*
            if (it->endFrame() < WINDOW_SIZE - 1)
            {
                feature.erase(it);
            }
            */
        }
    }

    void FeatureManager::removeBack() {
        for (auto it = feature.begin(), it_next = feature.begin();
             it != feature.end(); it = it_next) {
            it_next++;

            if (it->start_frame != 0)
                it->start_frame--;
            else {
                it->feature_per_frame.erase(it->feature_per_frame.begin());
                if (it->feature_per_frame.empty())
                    feature.erase(it);
            }
        }
    }

    void FeatureManager::removeFront(int frame_count) {
        for (auto it = feature.begin(), it_next = feature.begin();
            it != feature.end(); it = it_next) {
            it_next++;

            if (it->start_frame == frame_count) {
                it->start_frame--;
            } else {
                int j = WINDOW_SIZE - 1 - it->start_frame;
                if (it->endFrame() < frame_count - 1)
                    continue;
                it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
                if (it->feature_per_frame.empty())
                    feature.erase(it);
            }
        }
    }

    double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count) {
        //Check the second last frame is keyframe or not
        //by computing the parallax between second last frame and third last frame
        const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
        const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

        double ans = 0;
        Vector3d p_j = frame_j.point;

        double u_j = p_j(0);
        double v_j = p_j(1);

        Vector3d p_i = frame_i.point;
        Vector3d p_i_comp;

//        double dep_i = p_i(2);
        double u_i = p_i(0);
        double v_i = p_i(1);
        double du = u_i - u_j, dv = v_i - v_j;

//        ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));
        ans = sqrt(du * du + dv * dv);

        return ans;
    }
}