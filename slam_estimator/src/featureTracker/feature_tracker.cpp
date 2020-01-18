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

#include "feature_tracker.h"
#include <algorithm>
#ifdef SHOW_PROFILING
	#include "../utility/log/Profiler.hpp"
	#include "../utility/log/Logger.hpp"
#endif

/**
 * @namespace slam_estimator
 */

namespace slam_estimator {
    bool FeatureTracker::inBorder(const cv::Point2f &pt) {
        const int BORDER_SIZE = 1;
        int img_x = cvRound(pt.x);
        int img_y = cvRound(pt.y);
        return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
    }

    void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    void reduceVector(vector<int> &v, vector<uchar> status) {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    FeatureTracker::FeatureTracker() {
        stereo_cam = false;
        n_id = 0;
        hasPrediction = false;
        mask_updated = false;
    }

    void FeatureTracker::setMask() {
        mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

        // prefer to keep features that are tracked for long time
        vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

        for (size_t i = 0; i < cur_pts.size(); i++)
            cnt_pts_id.emplace_back(track_cnt[i], make_pair(cur_pts[i], ids[i]));


        sort(cnt_pts_id.begin(), cnt_pts_id.end(),
             [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b) {
                 return a.first > b.first;
             });

        cur_pts.clear();
        ids.clear();
        track_cnt.clear();

        for (auto &it : cnt_pts_id) {
            if (mask.at<uchar>(it.second.first) == 255) {
                cur_pts.push_back(it.second.first);
                ids.push_back(it.second.second);
                track_cnt.push_back(it.first);
                cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
            }
        }

        // Merge the dynamic object mask and long-term feature mask
//    if(CUBICLE) {
//        cv::bitwise_or(mask, dy_mask, final_mask);
//    } else
//        final_mask = mask;
        if (mask_updated) {
	        cv::bitwise_or(mask, dilate_mask_inv, mask);
	        mask_updated = false;
        }

    }

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time,
                                                                                        const cv::Mat &_img,
                                                                                        const cv::Mat &_img1,
                                                                                        const cv::Mat &_disp,
                                                                                        const cv::Mat &_mask) {
//        TicToc t_r;
        cur_time = _cur_time;
        cur_img = _img;
        cv::Mat erode_mask_;
        if (!_mask.empty()) {
            dy_mask = _mask;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));

            cv::erode(dy_mask, erode_mask_, element);
            cv::bitwise_and(cur_img, erode_mask_, cur_img);

            cv::bitwise_not(erode_mask_, dilate_mask_inv);
//            cv::dilate(dy_mask_inv, dilate_mask_inv, element);
	        mask_updated = true;
        }

        row = cur_img.rows;
        col = cur_img.cols;

        cur_pts.clear();
        track_status.clear();

        if (!prev_pts.empty()) {
            track_status.push_back(prev_pts.size()); // status 0: Total feature number
            vector<uchar> status;

#ifdef SHOW_PROFILING
	        utility::Timer t_o;
	        t_o.start();
#endif // SHOW_PROFILING

#ifdef GPU_FEATURE

//	        printf("gpu flow!\n");
                cv::cuda::GpuMat prev_gpu_img(prev_img);
                cv::cuda::GpuMat cur_gpu_img(cur_img);
                cv::cuda::GpuMat prev_gpu_pts(prev_pts);
                cv::cuda::GpuMat cur_gpu_pts(cur_pts);
                cv::cuda::GpuMat gpu_status;
                if (hasPrediction) {
                    cur_gpu_pts = cv::cuda::GpuMat(predict_pts);
                    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
                            cv::Size(21, 21), 1, 30, true);
                    d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);

                    vector<cv::Point2f> tmp_cur_pts(cur_gpu_pts.cols);
                    cur_gpu_pts.download(tmp_cur_pts);
                    cur_pts = tmp_cur_pts;

                    vector<uchar> tmp_status(gpu_status.cols);
                    gpu_status.download(tmp_status);
                    status = tmp_status;

                    int succ_num = 0;
                    for (unsigned char tmp_statu : tmp_status) {
                        if (tmp_statu)
                            succ_num++;
                    }
                    if (succ_num < 10) {
                        cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
                                cv::Size(21, 21), 3, 30, false);
                        d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);

                        vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
                        cur_gpu_pts.download(tmp1_cur_pts);
                        cur_pts = tmp1_cur_pts;

                        vector<uchar> tmp1_status(gpu_status.cols);
                        gpu_status.download(tmp1_status);
                        status = tmp1_status;
                    }
                } else {
                    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
                            cv::Size(21, 21), 3, 30, false);
                    d_pyrLK_sparse->calc(prev_gpu_img, cur_gpu_img, prev_gpu_pts, cur_gpu_pts, gpu_status);

                    vector<cv::Point2f> tmp1_cur_pts(cur_gpu_pts.cols);
                    cur_gpu_pts.download(tmp1_cur_pts);
                    cur_pts = tmp1_cur_pts;

                    vector<uchar> tmp1_status(gpu_status.cols);
                    gpu_status.download(tmp1_status);
                    status = tmp1_status;
                }
                if (FLOW_BACK) {
                    cv::cuda::GpuMat reverse_gpu_status;
                    cv::cuda::GpuMat reverse_gpu_pts = prev_gpu_pts;
                    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
                            cv::Size(21, 21), 1, 30, true);
                    d_pyrLK_sparse->calc(cur_gpu_img, prev_gpu_img, cur_gpu_pts, reverse_gpu_pts, reverse_gpu_status);

                    vector<cv::Point2f> reverse_pts(reverse_gpu_pts.cols);
                    reverse_gpu_pts.download(reverse_pts);

                    vector<uchar> reverse_status(reverse_gpu_status.cols);
                    reverse_gpu_status.download(reverse_status);

                    for (size_t i = 0; i < status.size(); i++) {
                        if (status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
                            status[i] = 1;
                        } else
                            status[i] = 0;
                    }
                }
//             printf("gpu temporal optical flow costs: %f ms\n",t_og.toc());

#else
	        vector<float> err;
	        if (hasPrediction) {
		        cur_pts = predict_pts;
		        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1,
		                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
		                                                  0.01),
		                                 cv::OPTFLOW_USE_INITIAL_FLOW);

		        int succ_num = 0;
		        for (unsigned char statu : status)
		        {
			        if (statu)
				        succ_num++;
		        }
		        if (succ_num < 10)
			        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21),
			                                 3);
	        }
	        else {
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
                int succ_num = 0;
                for (unsigned char statu : status)
                {
                    if (statu)
                        succ_num++;
                }
                if (succ_num < 10)
                    cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21),
                                             5);
		    }
	        // reverse check
	        if (FLOW_BACK) {
		        vector<uchar> reverse_status;
		        vector<cv::Point2f> reverse_pts = prev_pts;
		        cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err,
		                                 cv::Size(21, 21),
		                                 1,
		                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
		                                                  0.01),
		                                 cv::OPTFLOW_USE_INITIAL_FLOW);
		        for (size_t i = 0; i < status.size(); i++) {
			        if (status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
				        status[i] = 1;
			        } else
				        status[i] = 0;
		        }
	        }
//             printf("temporal optical flow costs: %fms\n", t_o.toc());
#endif

            for (int i = 0; i < int(cur_pts.size()); i++)
                if (status[i] && !inBorder(cur_pts[i]))
                    status[i] = 0;
//            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
#ifdef SHOW_PROFILING
	        t_o.stop();
	        WriteToLog("             [Feature track]Temporal optical flow costs", t_o);
#endif // SHOW_PROFILING
//        printf("track cnt %d\n", (int)ids.size());

        }

        track_status.push_back(track_cnt.size()); // status 1: cur tracked feature number


        for (auto &n : track_cnt)
            n++;

//        rejectOutliers();
#ifdef SHOW_PROFILING
	    utility::Timer t_m;
	    t_m.start();
#endif // SHOW_PROFILING

        setMask();

#ifdef SHOW_PROFILING
	    t_m.stop();
	    WriteToLog("             [Feature track]Set mask costs", t_m);
	    Logger::Write(printFullPrecision( utility::Timer::now() )  + "             [Feature track]Detect feature begins\n");
	    utility::Timer t_t;
	    t_t.start();
#endif // SHOW_PROFILING

        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());

#ifdef GPU_FEATURE
            if (n_max_cnt > 0) {
                if (mask.empty())
                    cout << "mask is empty " << endl;
//                if (mask.type() != CV_8UC1)
//                    cout << "mask type wrong " << endl;
//                TicToc t_g;
                cv::cuda::GpuMat cur_gpu_img(cur_img);
                cv::cuda::GpuMat d_prevPts;
//                TicToc t_gg;
                cv::cuda::GpuMat gpu_mask(mask);
                // printf("gpumat cost: %fms\n",t_gg.toc());
                cv::Ptr<cv::cuda::CornersDetector> detector = cv::cuda::createGoodFeaturesToTrackDetector(
                        cur_gpu_img.type(), n_max_cnt, 0.01, MIN_DIST);
                // cout << "new gpu points: "<< MAX_CNT - cur_pts.size()<<endl;
                detector->detect(cur_gpu_img, d_prevPts, gpu_mask);
                // std::cout << "d_prevPts size: "<< d_prevPts.size()<<std::endl;
                if (!d_prevPts.empty())
                    n_pts = cv::Mat_<cv::Point2f>(cv::Mat(d_prevPts));
                else
                    n_pts.clear();
                // sum_n += n_pts.size();
//                 printf("total point from gpu: %d\n",sum_n);
            } else
                n_pts.clear();
#else
	    if (n_max_cnt > 0) {
//                TicToc t_t_2;
		    if (mask.empty())
			    cout << "mask is empty " << endl;
//                if (mask.type() != CV_8UC1)
//                    cout << "mask type wrong " << endl;
		    cv::goodFeaturesToTrack(cur_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
		    // printf("good feature to track costs: %fms\n", t_t_2.toc());
//                std::cout << "n_pts size: "<< n_pts.size()<<std::endl;
	    } else
		    n_pts.clear();
	    // sum_n += n_pts.size();
	    // printf("total point from non-gpu: %d\n",sum_n);

#endif

#ifdef SHOW_PROFILING
	    t_t.stop();
	    WriteToLog("             [Feature track]Detect feature costs", t_t);
#endif // SHOW_PROFILING

        track_status.push_back(n_pts.size()); // status 2: Newly added feature number detected.

        for (auto &p : n_pts) {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }
        //printf("feature cnt after add %d\n", (int)ids.size());
//    }

        cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
        pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

        if ( stereo_cam) {
            ids_right.clear();
            cur_right_pts.clear();
            cur_un_right_pts.clear();
            right_pts_velocity.clear();
            cur_un_right_pts_map.clear();

            if (!cur_pts.empty()) {
//            printf("stereo image; track feature on right image\n");

                vector<uchar> status, statusRightLeft;

                if(!_disp.empty() && DISPARITY) {
                    const cv::Mat& dispImg = _disp;
//                    TicToc t_check;
//                    status.reserve(cur_pts.size());
                    // For each cur_pt, find its disparity using dispImg (as a lookup table)
                    for (auto & cur_pt : cur_pts) {
                        cv::Point2f right_pt;
                        int dist = static_cast<int>(dispImg.at<uchar>(cur_pt));
                        float right_x = cur_pt.x - float(dist);
//                        cout << "  | d: " << dist << "  | right: " << right_x << endl;
                        right_pt = cv::Point2f(right_x, cur_pt.y);
                        if (dist > 7 && dist < 128 && inBorder(right_pt)) {
                            cur_right_pts.push_back(right_pt);
                            status.push_back(1);
                        } else {
                            cur_right_pts.emplace_back(1, cur_pt.y);
                            status.push_back(0);
                        } // right_x out of border of image
                    }
//                    printf("Disparity cost %fms\n",t_check.toc());
                } else if (!_img1.empty()) {
                    vector<cv::Point2f> reverseLeftPts;
                    const cv::Mat& rightImg = _img1;

                    // This is to equalize the histogram of whole image in case of huge illumination change.
                    /*
                    {
                        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
                        clahe->apply(cur_img, cur_img);
                        if(!rightImg.empty())
                            clahe->apply(rightImg, rightImg);
                    }
                    */
#ifdef GPU_FEATURE
                    TicToc t_og1;
                    cv::cuda::GpuMat cur_gpu_img(cur_img);
                    cv::cuda::GpuMat right_gpu_Img(rightImg);
                    cv::cuda::GpuMat cur_gpu_pts(cur_pts);
                    cv::cuda::GpuMat cur_right_gpu_pts;
                    cv::cuda::GpuMat gpu_status;
                    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
                            cv::Size(21, 21), 3, 30, false);
                    d_pyrLK_sparse->calc(cur_gpu_img, right_gpu_Img, cur_gpu_pts, cur_right_gpu_pts, gpu_status);

                    vector<cv::Point2f> tmp_cur_right_pts(cur_right_gpu_pts.cols);
                    cur_right_gpu_pts.download(tmp_cur_right_pts);
                    cur_right_pts = tmp_cur_right_pts;

                    vector<uchar> tmp_status(gpu_status.cols);
                    gpu_status.download(tmp_status);
                    status = tmp_status;

                    if (FLOW_BACK) {
                        cv::cuda::GpuMat reverseLeft_gpu_Pts;
                        cv::cuda::GpuMat status_gpu_RightLeft;
                        cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
                                cv::Size(21, 21), 3, 30, false);
                        d_pyrLK_sparse->calc(right_gpu_Img, cur_gpu_img, cur_right_gpu_pts, reverseLeft_gpu_Pts,
                                             status_gpu_RightLeft);

                        vector<cv::Point2f> tmp_reverseLeft_Pts(reverseLeft_gpu_Pts.cols);
                        reverseLeft_gpu_Pts.download(tmp_reverseLeft_Pts);
                        reverseLeftPts = tmp_reverseLeft_Pts;

                        vector<uchar> tmp1_status(status_gpu_RightLeft.cols);
                        status_gpu_RightLeft.download(tmp1_status);
                        statusRightLeft = tmp1_status;
                        for (size_t i = 0; i < status.size(); i++) {
                            if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) &&
                                distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                                status[i] = 1;
                            else
                                status[i] = 0;
                        }
                    }
//                 printf("gpu left right optical flow cost %fms\n",t_og1.toc());
#else
//	            TicToc t_check;
                    vector<float> err;
                    // cur left ---- cur right
                    cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21),
                                             3);
                    // reverse check cur right ---- cur left
                    if (FLOW_BACK) {
                        cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err,
                                                 cv::Size(21, 21), 3);
                        for (size_t i = 0; i < status.size(); i++) {
                            if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) &&
                                distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                                status[i] = 1;
                            else
                                status[i] = 0;
                        }
                    }
//                 printf("left right optical flow cost %fms\n",t_check.toc());
#endif

                }

                ids_right = ids;
                reduceVector(cur_right_pts, status);
                reduceVector(ids_right, status);
                // only keep left-right pts
                /*
                reduceVector(cur_pts, status);
                reduceVector(ids, status);
                reduceVector(track_cnt, status);
                reduceVector(cur_un_pts, status);
                reduceVector(pts_velocity, status);
                */
//                cout << "1 before undistort" << endl;
                cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
//                cout << "2 after undistort" << endl;
                right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map,
                                                 prev_un_right_pts_map);
//                cout << "3 after velocity" << endl;
            }
            prev_un_right_pts_map = cur_un_right_pts_map;
        }
        if (SHOW_TRACK)
            drawTrack(cur_img, ids, cur_pts);

        prev_img = cur_img;
        prev_pts = cur_pts;
        prev_un_pts = cur_un_pts;
        prev_un_pts_map = cur_un_pts_map;
        prev_time = cur_time;
        hasPrediction = false;

        prevLeftPtsMap.clear();
        for (size_t i = 0; i < cur_pts.size(); i++)
            prevLeftPtsMap[ids[i]] = cur_pts[i];

        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
        for (size_t i = 0; i < ids.size(); i++) {
            int feature_id = ids[i];
            double x, y, z;
            // Normalized coordinate
            x = cur_un_pts[i].x;
            y = cur_un_pts[i].y;
            z = 1;
            double p_u, p_v;
	        // Image plane coordinate
            p_u = cur_pts[i].x;
            p_v = cur_pts[i].y;
            int camera_id = 0;
            double velocity_x, velocity_y;
            velocity_x = pts_velocity[i].x;
            velocity_y = pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        }

        if (stereo_cam) {
            for (size_t i = 0; i < ids_right.size(); i++) {
                int feature_id = ids_right[i];
                double x, y, z;
                x = cur_un_right_pts[i].x;
                y = cur_un_right_pts[i].y;
                z = 1;
                double p_u, p_v;
                p_u = cur_right_pts[i].x;
                p_v = cur_right_pts[i].y;
                int camera_id = 1;
                double velocity_x, velocity_y;
                velocity_x = right_pts_velocity[i].x;
                velocity_y = right_pts_velocity[i].y;

                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
        }

//    printf("feature track whole time %f\n", t_r.toc());
        return featureFrame;
    }

    void FeatureTracker::rejectOutliers() {
        if (cur_pts.size() >= 8) {
#ifdef SHOW_PROFILING
	        utility::Timer t_f;
	        t_f.start();
	        Logger::Write("             [Feature track]FM RANSAC begins");
#endif // SHOW_PROFILING
            vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
            for (size_t i = 0; i < cur_pts.size(); i++) {
                Eigen::Vector3d tmp_p;
                m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
                un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

                m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
                tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
                tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
                un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            }

            vector<uchar> status;
            cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
            int size_a = cur_pts.size();
            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(cur_un_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
#ifdef SHOW_PROFILING
	        t_f.stop();
	        Logger::Write("             [Feature track]FM ransac: " + std::to_string(size_a) +
	        " -> " + std::to_string(cur_pts.size()) + " : " + std::to_string(1.0 * cur_pts.size() / size_a) + "\n");
	        WriteToLog("             [Feature track]FM ransac costs", t_f);
#endif // SHOW_PROFILING
        }
    }

    void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file) {
        for (const auto &i : calib_file) {
//            ROS_INFO("reading parameter of camera %s", i.c_str());
//            camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(i);
	        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromCalibInfo(i);
            m_camera.push_back(camera);
        }
        if (calib_file.size() == 2)
            stereo_cam = true;
    }

    void FeatureTracker::showUndistortion(const string &name) {
        cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
        vector<Eigen::Vector2d> distortedp, undistortedp;
        for (int i = 0; i < col; i++)
            for (int j = 0; j < row; j++) {
                Eigen::Vector2d a(i, j);
                Eigen::Vector3d b;
                m_camera[0]->liftProjective(a, b);
                distortedp.push_back(a);
                undistortedp.emplace_back(b.x() / b.z(), b.y() / b.z());
                //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
            }
        for (int i = 0; i < int(undistortedp.size()); i++) {
            cv::Mat pp(3, 1, CV_32FC1);
            pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
            pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
            pp.at<float>(2, 0) = 1.0;
            //cout << trackerData[0].K << endl;
            //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
            //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
            if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 &&
                pp.at<float>(0, 0) + 300 >= 0 &&
                pp.at<float>(0, 0) + 300 < col + 600) {
                undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(
                        distortedp[i].y(), distortedp[i].x());
            } else {
                //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
            }
        }
        cv::imshow(name, undistortedImg);
        cv::waitKey(0);
    }

    vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam) {
        vector<cv::Point2f> un_pts;
        for (auto &pt : pts) {
            Eigen::Vector2d a(pt.x, pt.y);
            Eigen::Vector3d b;
            cam->liftProjective(a, b);
            un_pts.emplace_back(b.x() / b.z(), b.y() / b.z());
        }
        return un_pts;
    }

    vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids_, vector<cv::Point2f> &pts,
                                                    map<int, cv::Point2f> &cur_id_pts,
                                                    map<int, cv::Point2f> &prev_id_pts) {
        vector<cv::Point2f> pts_velocity_;
        cur_id_pts.clear();
        for (size_t i = 0; i < ids_.size(); i++) {
            cur_id_pts.insert(make_pair(ids_[i], pts[i]));
        }

        // calculate points velocity
        if (!prev_id_pts.empty()) {
            double dt = cur_time - prev_time;

            for (size_t i = 0; i < pts.size(); i++) {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_id_pts.find(ids_[i]);
                if (it != prev_id_pts.end()) {
                    double v_x = (pts[i].x - it->second.x) / dt;
                    double v_y = (pts[i].y - it->second.y) / dt;
                    pts_velocity_.emplace_back(v_x, v_y);
                } else
                    pts_velocity_.emplace_back(0, 0);

            }
        } else {
            for (size_t i = 0; i < cur_pts.size(); i++) {
                pts_velocity_.emplace_back(0, 0);
            }
        }
        return pts_velocity_;
    }

    void FeatureTracker::drawTrack_stereo(const cv::Mat &imLeft, const cv::Mat &imRight,
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts,
                                   vector<cv::Point2f> &curRightPts) {
        //int rows = imLeft.rows;
//     int cols = imLeft.cols;
//    if (!imRight.empty() && stereo_cam)
//        cv::hconcat(imLeft, imRight, imTrack);
//    else
//        imTrack = imLeft.clone();
//    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);
//
//    for (size_t j = 0; j < curLeftPts.size(); j++)
//    {
//        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
//        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
//    }
//    if (!imRight.empty() && stereo_cam)
//    {
//        for (size_t i = 0; i < curRightPts.size(); i++)
//        {
//            cv::Point2f rightPt = curRightPts[i];
//            rightPt.x += cols;
//            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
////            cv::Point2f leftPt = curLeftPts[i];
////            cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
//        }
//    }

        imTrack = imLeft.clone();
        cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);
        for (auto & curLeftPt : curLeftPts) {
//        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
//        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            cv::circle(imTrack, curLeftPt, 2, cv::Scalar(255, 0, 0), 2);
        }

        map<int, cv::Point2f>::iterator mapIt;
        for (size_t i = 0; i < curLeftIds.size(); i++) {
            int id = curLeftIds[i];
            mapIt = prevLeftPtsMap.find(id);
            if (mapIt != prevLeftPtsMap.end()) {
                cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
            }
        }

        //draw prediction
        /*
        for(size_t i = 0; i < predict_pts_debug.size(); i++)
        {
            cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
        }
        */
        //printf("predict pts size %d \n", (int)predict_pts_debug.size());


        //cv::Mat imCur2Compress;
        //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));

//    cv::imshow("tracking", imTrack);
//    cv::imshow("mask", mask);
//    cv::imshow("mask object", dy_mask);
//    cv::imshow("final mask", dilate_mask_inv);
//    cv::waitKey(2);
    }

    void FeatureTracker::drawTrack(const cv::Mat &imLeft,
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts) {
        //int rows = imLeft.rows;
//     int cols = imLeft.cols;
        cv::Mat imTmp;
        imTmp = imLeft.clone();
        cv::cvtColor(imTmp, imTmp, CV_GRAY2RGB);
        for (auto & curLeftPt : curLeftPts) {
//        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
//        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
            cv::circle(imTmp, curLeftPt, 2, cv::Scalar(255, 0, 0), 2);
        }

        map<int, cv::Point2f>::iterator mapIt;
        for (size_t i = 0; i < curLeftIds.size(); i++) {
            int id = curLeftIds[i];
            mapIt = prevLeftPtsMap.find(id);
            if (mapIt != prevLeftPtsMap.end()) {
                cv::arrowedLine(imTmp, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
            }
        }

        //draw prediction
        /*
        for(size_t i = 0; i < predict_pts_debug.size(); i++)
        {
            cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
        }
        */
        //printf("predict pts size %d \n", (int)predict_pts_debug.size());

        // Draw feature detection & tracking status information
        stringstream s;
        s << " Total Features  " << track_status[0] << " | Tracked Features  " << track_status[1]
        << " | New Features  " << track_status[2];

        int baseline = 4;
        cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 3.0, 2, &baseline);

        imTrack = cv::Mat(row + textSize.height + 10, col, imTmp.type());
        imTmp.copyTo(imTrack.rowRange(0, row).colRange(0, col));
        imTrack.rowRange(row, imTrack.rows) = cv::Mat::zeros(textSize.height + 14, col, imTmp.type());
        cv::putText(imTrack, s.str(), cv::Point(5, imTrack.rows - 5),
                cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);

    }

    void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts) {
        hasPrediction = true;
//        printf("predict_pts size: %lu", predict_pts.size());
        predict_pts.clear();
        map<int, Eigen::Vector3d>::iterator itPredict;
        for (size_t i = 0; i < ids.size(); i++) {
            //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
            int id = ids[i];
            itPredict = predictPts.find(id);
            if (itPredict != predictPts.end()) {
                Eigen::Vector2d tmp_uv;
                m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
                predict_pts.emplace_back(tmp_uv.x(), tmp_uv.y());
            } else
                predict_pts.push_back(prev_pts[i]);
        }
    }


    void FeatureTracker::removeOutliers(set<int> &removePtsIds) {
        std::set<int>::iterator itSet;
        vector<uchar> status;
        for (int id : ids) {
            itSet = removePtsIds.find(id);
            if (itSet != removePtsIds.end())
                status.push_back(0);
            else
                status.push_back(1);
        }

        reduceVector(prev_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
    }


    cv::Mat FeatureTracker::getTrackImage() {
        return imTrack;
    }
}