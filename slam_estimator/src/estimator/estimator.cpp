/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file estimator.cpp
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief SLAM main process of SSLAM-estimator.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/
#include "estimator.h"
#include "../utility/visualization.h"
#ifdef SHOW_PROFILING
	#include "../utility/log/Profiler.hpp"
	#include "../utility/log/Logger.hpp"
#endif

/**
 * @namespace slam_estimator
 */
namespace slam_estimator {
    Estimator::Estimator() :
    count_(0), f_manager{Rs}, gps_bad(false), gps_rec(false), initGPS(false) {
        ROS_INFO("Init of VO estimation");
        initThreadFlag = false;
        clearState();
        last_time = 0;
        cov_position = Eigen::Matrix3d::Zero();
        cvNamedWindow("SLAM tracking", CV_WINDOW_NORMAL);
        cvMoveWindow("SLAM tracking", 0, 455);
        cvResizeWindow("SLAM tracking", 720, 478);
    }

    Estimator::~Estimator() {
        if (MULTIPLE_THREAD)
            processThread.join();
    }

    void Estimator::clearState() {
        mProcess.lock();
        while (!gpsBuf.empty())
            gpsBuf.pop();
        while (!accBuf.empty())
            accBuf.pop();
        while (!gyrBuf.empty())
            gyrBuf.pop();
        while (!spdBuf.empty())
            spdBuf.pop();
        while (!angBuf.empty())
            angBuf.pop();
        while (!featureBuf.empty())
            featureBuf.pop();
        while (!heightBuf.empty())
            heightBuf.pop();

        prevTime = -1;
        curTime = 0;
        openExEstimation = false;
        last_vec_rev = Eigen::Vector3d(0, 0, 0);
        last_ang_rev = Eigen::Quaterniond::Identity();
        inputImageCnt = 0;
        initFirstPoseFlag = false;

        for (int i = 0; i < WINDOW_SIZE + 1; i++) {
            Rs[i].setIdentity();
            Ps[i].setZero();
            Vs[i].setZero();
            Bas[i].setZero();
            Bgs[i].setZero();
            dt_buf[i].clear();
            t_buf[i].clear();
            gt_buf[i].clear();
            linear_acceleration_buf[i].clear();
            angular_velocity_buf[i].clear();
            linear_speed_buf[i].clear();
            angular_read_buf[i].clear();
            gps_buf[i].clear();
            gps_status[i].clear();
            sum_dt[i] = 0;

            delete pre_integrations[i];
            pre_integrations[i] = nullptr;
        }

        for (int i = 0; i < NUM_OF_CAM; i++) {
            tic[i] = Vector3d::Zero();
            ric[i] = Matrix3d::Identity();
        }

        first_imu = false;
        first_gps = false;
        sum_of_back = 0;
        sum_of_front = 0;
        frame_count = 0;
        solver_flag = INITIAL;
        initial_timestamp = 0;
        all_image_frame.clear();

        delete tmp_pre_integration;
        delete last_marginalization_info;

        tmp_pre_integration = nullptr;
        last_marginalization_info = nullptr;
        last_marginalization_parameter_blocks.clear();

        f_manager.clearState();
        failure_occur = false;

        mProcess.unlock();
    }

    void Estimator::setParameter() {
        mProcess.lock();
        for (int i = 0; i < NUM_OF_CAM; i++) {
            tic[i] = TIC[i];
            ric[i] = RIC[i];
            cout << " extrinsic cam " << i << endl << ric[i] << endl << tic[i].transpose() << endl;
        }
        f_manager.setRic(ric);
        ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
        ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
        ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
        td = TD;
        g = G;
//        cout << "set g " << g.transpose() << endl;
        featureTracker.readIntrinsicParameter(CAM_NAMES);
        mProcess.unlock();
    }

    void Estimator::startProcessThread() {
        std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
        if (MULTIPLE_THREAD && !initThreadFlag) {
            initThreadFlag = true;
            processThread = std::thread(&Estimator::processMeasurements, this);
        }
    }

    void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1,
                               const cv::Mat &_disp, const cv::Mat &_mask) {
        inputImageCnt++;
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
#ifdef SHOW_PROFILING
	    utility::Timer featureTrackerTime;
	    featureTrackerTime.start();
#endif // SHOW_PROFILING
        if (_mask.empty()) {
            featureFrame = featureTracker.trackImage(t, _img, _img1, _disp);
        } else {
            featureFrame = featureTracker.trackImage(t, _img, _img1, _disp, _mask);
        }

#ifdef SHOW_PROFILING
	    featureTrackerTime.stop();
	    WriteToLog("  Whole featureTracker costs  ", featureTrackerTime);
#endif // SHOW_PROFILING

        if (SHOW_TRACK) {
            cv::Mat imgTrack = featureTracker.getTrackImage();
            cv::imshow("SLAM tracking", imgTrack);
//            cv::Mat imgMask = featureTracker.getMaskImage();
//            cv::imshow("mask", imgMask);
            cv::waitKey(1);
//            pubTrackImage(imgTrack, t);
        }

        if (MULTIPLE_THREAD) {
//            if (inputImageCnt % 2 == 0) {
                mBuf.lock();
                featureBuf.push(make_pair(t, featureFrame));
                mBuf.unlock();
//            }
        } else {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
            TicToc processTime;
            processMeasurements();
            printf("process time: %f\n", processTime.toc());
        }
    }

    void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity) {
        mBuf.lock();
        accBuf.push(make_pair(t, linearAcceleration));
        gyrBuf.push(make_pair(t, angularVelocity));
        //printf("input imu with time %f \n", t);
        mBuf.unlock();

//        if (solver_flag == NON_LINEAR) {
//            mPropagate.lock();
//            fastPredictIMU(t, linearAcceleration, angularVelocity);
//            pubLatestOdometry(latest_P, latest_Q, latest_V, t);
//            mPropagate.unlock();
//        }
    }

    void
    Estimator::inputINS(double t, const Vector3d &linearSpeed, const Quaterniond &angularRead, const double height) {
        mBuf.lock();
        spdBuf.push(make_pair(t, linearSpeed));
        angBuf.push(make_pair(t, angularRead));
        heightBuf.push(make_pair(t, height));
//    printf("input ins with time %f \n", t);
        mBuf.unlock();

//    fastPredictINS(t, linearSpeed, angularRead);
//    if (solver_flag == NON_LINEAR)
//        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
    }

    void
    Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame) {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();

        if (!MULTIPLE_THREAD)
            processMeasurements();
    }

    void Estimator::inputGPS(double t, double x, double y, double z, double posAccuracy_x, double posAccuracy_y) {
        mBuf.lock();
        if(!initGPS) {
            offset.x() = x;
            offset.y() = y;
            x = 0;
            y = 0;
            initGPS = true;
        } else {
            x = x - offset.x();
            y = y - offset.y();
        }
        Vector5d gpsPos;
        gpsPos << x, y, z, posAccuracy_x, posAccuracy_y;

        gpsBuf.push(make_pair(t, gpsPos));
        mBuf.unlock();
    }

    bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                                   vector<pair<double, Eigen::Vector3d>> &gyrVector) {
        if (accBuf.empty()) {
            printf("Cannot receive imu\n");
            return false;
        }
        //printf("get imu from %f %f\n", t0, t1);
        //printf("imu front time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
        if (t1 <= accBuf.back().first) {
            while (accBuf.front().first <= t0) {
                accBuf.pop();
                gyrBuf.pop();
            }
            while (accBuf.front().first < t1) {
                accVector.push_back(accBuf.front());
                accBuf.pop();
                gyrVector.push_back(gyrBuf.front());
                gyrBuf.pop();
            }
            accVector.push_back(accBuf.front());
            gyrVector.push_back(gyrBuf.front());
        } else {
            printf("wait for imu\n");
            return false;
        }
        return true;
    }

    bool Estimator::getINSInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &spdVector,
                                   vector<pair<double, Eigen::Quaterniond>> &angVector,
                                   vector<pair<double, double>> &heightVector) {
        if (spdBuf.empty()) {
            printf("Cannot receive INS.\n");
            return false;
        }
//    printf("get ins from %f %f\n", t0, t1);
//    printf("ins front time %f   ins end time %f\n", spdBuf.front().first, spdBuf.back().first);
        if (t1 <= spdBuf.back().first) {
            while (spdBuf.front().first <= t0) {
                spdBuf.pop();
                angBuf.pop();
                heightBuf.pop();
            }
//        printf("t1: %f\n", t1);
            while (spdBuf.front().first < t1) {
//            printf("ins0: %f\n", spdBuf.front().first);
                spdVector.push_back(spdBuf.front());
                spdBuf.pop();
                angVector.push_back(angBuf.front());
                angBuf.pop();
                heightVector.push_back(heightBuf.front());
                heightBuf.pop();
            }
            if (!spdBuf.empty()) {
//            printf("ins1: %f\n", spdBuf.front().first);
                spdVector.push_back(spdBuf.front());
                angVector.push_back(angBuf.front());
                heightVector.push_back(heightBuf.front());
            }
        } else {
            printf("wait for INS info!\n");
            return false;
        }
        return true;
    }

    bool Estimator::getGPSInterval(double t0, double t1, vector<pair<double, Vector5d>> &gpsVector) {
        if (gpsBuf.empty()) {
            printf("Cannot receive GPS.\n");
            return false;
        }
        if (t1 <= gpsBuf.back().first) {
            while (gpsBuf.front().first <= t0) {
                gpsBuf.pop();
            }
//        printf("t1: %f\n", t1);
            while (gpsBuf.front().first < t1) {
//            printf("gps0: %f\n", gpsBuf.front().first);
                gpsVector.emplace_back(gpsBuf.front());
                gpsBuf.pop();
            }
            if (!gpsBuf.empty()) {
//            printf("gps1: %f\n", gpsBuf.front().first);
                gpsVector.emplace_back(gpsBuf.front());
            }
        } else {
            printf("wait for GPS info!\n");
            return false;
        }
        return true;
    }

    bool Estimator::IMUAvailable(double t) {
        return !accBuf.empty() && t <= accBuf.back().first;
    }

    bool Estimator::INSAvailable(double t) {
        return !spdBuf.empty() && t <= spdBuf.back().first;
//    return !spdBuf.empty() ;
    }

    bool Estimator::GPSAvailable(double t) {

        return !gpsBuf.empty() && t <= gpsBuf.back().first;
    }

    void Estimator::processMeasurements() {
        while (processThread_swt) {
            pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
            vector<pair<double, Eigen::Vector3d>> accVector, gyrVector, spdVector;
            vector<pair<double, Vector5d>> gpsVector;
            vector<pair<double, Eigen::Quaterniond>> angVector;
            vector<pair<double, double>> heightVector;

            if (!featureBuf.empty()) {
                feature = featureBuf.front();
                curTime = feature.first + td;
                if (USE_IMU) {
                    while (true) {
                        if (processThread_swt == false)
                            break;
                        if (IMUAvailable(curTime))
                            break;
                        else {
                            printf("Wait for imu ... \n");
                            if (!MULTIPLE_THREAD)
                                return;
                            std::chrono::milliseconds dura(5);
                            std::this_thread::sleep_for(dura);
                        }
                    }
                } else if (USE_INS) {
                    while (true) {
                        if (processThread_swt == false)
                            break;
                        if (INSAvailable(curTime))
                            break;
                        else {
//                            printf("Wait for inspva ... \n");
                            if (!MULTIPLE_THREAD)
                                return;
                            std::chrono::milliseconds dura(10);
                            std::this_thread::sleep_for(dura);
                        }
                    }
                }
				int cnt_gps_ = 0;
                if(USE_GPS) {
                    while (cnt_gps_ < 3) {
                        if (processThread_swt == false)
                            break;
                        if (GPSAvailable(curTime)) {
                        	gps_rec = true;
	                        break;
                        } else {
                        	cnt_gps_ ++;
                            printf("Wait for gps position ... \n");
                            gps_rec = false;
                            if (!MULTIPLE_THREAD)
                                return;
                            std::chrono::milliseconds dura(5);
                            std::this_thread::sleep_for(dura);
                        }
                    }
                }

                mBuf.lock();
                if (USE_IMU)
                    getIMUInterval(prevTime, curTime, accVector, gyrVector);
                else if (USE_INS)
                    getINSInterval(prevTime, curTime, spdVector, angVector, heightVector);

                if (USE_GPS)
                    if(!getGPSInterval(prevTime, curTime, gpsVector))
                        gps_rec = false;

                featureBuf.pop();
                mBuf.unlock();

                if (USE_IMU) {
                    if (!initFirstPoseFlag)
                        initFirstIMUPose(accVector);
                    for (size_t i = 0; i < accVector.size(); i++) {
                        double dt;
                        if (i == 0)
                            dt = accVector[i].first - prevTime;
                        else if (i == accVector.size() - 1)
                            dt = curTime - accVector[i - 1].first;
                        else
                            dt = accVector[i].first - accVector[i - 1].first;
                        processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                    }
                } else if (USE_INS) {
                    if (!initFirstPoseFlag)
                        initFirstINSPose(spdVector, angVector, heightVector);
                    bool lastone = false;
//                printf("size: %d\n", spdVector.size());
                    for (size_t i = 0; i < spdVector.size(); i++) {
                        double dt;
                        if (i == 0)
                            dt = spdVector[i].first - prevTime;
                        else if (i == spdVector.size() - 1) {
                            dt = curTime - spdVector[i - 1].first;
                            lastone = true;
                        } else {
                            dt = spdVector[i].first - spdVector[i - 1].first;
                            lastone = false;
                        }

                            processINS(spdVector[i].first, dt, spdVector[i].second, angVector[i].second,
                                   heightVector[i].second, lastone);

                    }
                }

                if (USE_GPS && gps_rec) {
                    bool lastone = false;

                    for (size_t i = 0; i < gpsVector.size(); i++) {
                        double dt;
                        if (i == 0)
                            dt = gpsVector[i].first - prevTime;
                        else if (i == gpsVector.size() - 1) {
                            dt = curTime - gpsVector[i - 1].first;
                            lastone = true;
                        } else {
                            dt = gpsVector[i].first - gpsVector[i - 1].first;
                            lastone = false;
                        }

                        processGPS(gpsVector[i].first, dt, gpsVector[i].second, lastone);
                    }
                }

                mProcess.lock();
                PoseSolver(feature.second, feature.first);
                prevTime = curTime;

                std_msgs::Header header;
                header.frame_id = "world";
                header.stamp = ros::Time(feature.first);
//            header.stamp = ros::Time::now();

                printStatistics(*this);
                pubOdometry(*this, header);
//            pubKeyPoses(*this, header);
                pubCameraPose(*this, header);
                pubPointCloud(*this, header);
                pubKeyframe(*this);
//                pubTF(*this, header);
                mProcess.unlock();
                last_time = header.stamp.toSec();
            }

            if (!MULTIPLE_THREAD)
                break;

            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }

    void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector) {
        printf("init first imu pose\n");
        initFirstPoseFlag = true;
        //return;
        Eigen::Vector3d averAcc(0, 0, 0);
        int n = (int) accVector.size();
        for (auto &i : accVector) {
            averAcc = averAcc + i.second;
        }
        averAcc = averAcc / n;
        printf("average acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
        Matrix3d R0 = Utility::g2R(averAcc);
        double yaw = Utility::R2ypr(R0).x();
        R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
        Rs[0] = R0;
        cout << "init R0 " << endl << Rs[0] << endl;
        //Vs[0] = Vector3d(5, 0, 0);
    }

    void Estimator::initFirstINSPose(vector<pair<double, Eigen::Vector3d>> &spdVector,
                                     vector<pair<double, Eigen::Quaterniond>> &angVector,
                                     vector<pair<double, double>> heightVector) {
        printf("init first INS pose\n");
        initFirstPoseFlag = true;
        //TODO: Even first frame needs interpolation
        Eigen::Quaterniond latest_ang = angVector.back().second;

//    printf("average acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
        Vs[0] = spdVector.back().second;
        Rs[0] = latest_ang.toRotationMatrix();
        cout << "init R0 " << endl << Rs[0] << endl;
        Ps[0].z() = heightVector.back().second;
    }

    void Estimator::initFirstPose(const Eigen::Vector3d &p, const Eigen::Matrix3d &r) {
        Ps[0] = p;
        Rs[0] = r;
    }

    void
    Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity) {
        if (!first_imu) {
            first_imu = true;
            acc_0 = linear_acceleration;
            gyr_0 = angular_velocity;
        }

        if (!pre_integrations[frame_count]) {
            pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
        }
        if (frame_count != 0) {
            pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
            //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

            dt_buf[frame_count].push_back(dt);
            linear_acceleration_buf[frame_count].push_back(linear_acceleration);
            angular_velocity_buf[frame_count].push_back(angular_velocity);

            int j = frame_count;
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
            Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
            Vs[j] += dt * un_acc;
        }
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    void Estimator::processINS(double t, double dt, const Vector3d &linear_speed,
                               const Quaterniond &angular_read, const double height_, const bool last_) {
        if (!first_imu) {
            first_imu = true;
//        spd_0 = linear_speed;
//        ang_0 = angular_read;
	      Ps[0].z() = height_;
//	      height_read_buf[0].push_back(height_);
        }
//    cout << "frame count: " << frame_count << endl;
        if (frame_count != 0) {

            int j = frame_count;

            if (last_) {

                double scale = dt / (t - t_buf[j].back());
                assert(scale <= 1 && scale >= 0);
//            cout << "scale: " << scale << endl;
                Eigen::Quaterniond angular_interp;
                Eigen::Vector3d speed_interp;
                angular_interp = angular_read_buf[j].back().slerp(scale, angular_read);
                angular_read_buf[j].push_back(angular_interp);
                Rs[j] = angular_interp.toRotationMatrix();

                speed_interp = linear_speed_buf[j].back() + scale * (linear_speed - linear_speed_buf[j].back());
                linear_speed_buf[j].push_back(speed_interp);
                Vs[j] = speed_interp;

            } else {

                angular_read_buf[j].push_back(angular_read);
                Rs[j] = angular_read.toRotationMatrix();

                Vs[j] = linear_speed;
                linear_speed_buf[j].push_back(linear_speed);
            }

            dt_buf[j].push_back(dt);
            Ps[j] += Vs[j] * dt;
            sum_dt[j] += dt;
            t_buf[j].push_back(t);
//	        height_read_buf[j].push_back(height_);
	        Ps[j].z() = height_;
        }
    }

    void Estimator::processGPS(double t, double dt, const Vector5d &gps_position, const bool last_) {
        // gps_stat_ refers to bad gps status if its value is 1; otherwise means good condition.
        bool gps_stat_ = gps_position[3] > 0.0052 || gps_position[4] > 0.0052 || !gps_rec;
    	if (!first_gps) {
            first_gps = true;
            gps_buf[0].push_back(gps_position);
            gps_status[0].push_back(gps_stat_);
        }
//    cout << "frame count: " << frame_count << endl;
        if (frame_count != 0) {
            int j = frame_count;

            if (last_) {
	            Vector5d position_interp;
                double scale = dt / (t - gt_buf[j].back());
                position_interp = scale * (gps_position - gps_buf[j].back());
                gps_buf[j].emplace_back(gps_buf[j].back() + position_interp);
                gps_buf[j].back()[3] = gps_position[3];
                gps_buf[j].back()[4] = gps_position[4];

            } else {
                gps_buf[j].push_back(gps_position);
            }
	        gps_status[j].push_back(gps_stat_);
//            linear_speed_buf[j].push_back(speed_interp);

            gt_buf[j].push_back(t);
            Ps[j].z() = gps_position[2];
        }
    }

/**
 * @brief   Process the data of image and solve pose estimation
 * @Description addFeatureCheckParallax() adds feature points into feature,
 *              calculates tracking times and parallax to judge whether inserting key frame,
 *              calibrating external params,
 *              jointly initialize or VIO using sliding window nonlinear optimization
 * @param[in]   image map of all the feature points of an image[camera_id,[x,y,z,u,v,vx,vy]], index is feature_id
 * @param[in]   header header information
*/
    void
    Estimator::PoseSolver(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                            const double header) {
#ifdef SHOW_PROFILING
		Logger::Write( "#   new image ---------------------------- \n" );
		Logger::Write( "    Adding feature points: " + std::to_string(image.size()) + "\n" );
#endif
        // Judge whether this is a keyframe by calculating tracking times and parallax.
        if (f_manager.addFeatureCheckParallax(frame_count, image, td)) {
        	// If last new frame is a keyframe, we marginalize the last keyframe in the sliding window.
            marginalization_flag = MARGIN_OLD;
//        printf("keyframe\n");
        } else {
	        // If last new frame is not a keyframe, we consider it is similar to current new frame,
	        // so we throw away the second newest frame, which will not lose useful information of
	        // constraints of current frame and its landmarks.
            marginalization_flag = MARGIN_SECOND_NEW;
//        printf("non-keyframe\n");
        }
#ifdef SHOW_PROFILING
		Logger::Write( "    " + std::string(marginalization_flag ? "Non-keyframe" : "Keyframe") + "\n" );
		Logger::Write( "    Frame count: " + std::to_string(frame_count) + "\n" );
		Logger::Write( "    Number of features:: " + std::to_string(f_manager.getFeatureCount()) + "\n" );
#endif
        Headers[frame_count] = header;

        ImageFrame imageframe(image, header);
        imageframe.pre_integration = tmp_pre_integration;
        all_image_frame.insert(make_pair(header, imageframe));
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0,
                                                  Bas[frame_count], Bgs[frame_count]};
        if (ESTIMATE_EXTRINSIC == 2) {
            ROS_INFO("calibrating extrinsic param, rotation movement is needed");
            if (frame_count > 0) {
                vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
                Matrix3d calib_ric;
                if (USE_IMU) {
                    if (initial_ex_rotation.CalibrationExRotation(corres,
                    		pre_integrations[frame_count]->delta_q, calib_ric)) {
                        ROS_WARN("initial extrinsic rotation calibration success");
                        ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                        ric[0] = calib_ric;
                        RIC[0] = calib_ric;
                        ESTIMATE_EXTRINSIC = 1;
                    }
                } else if (USE_INS) {
                    Quaterniond delta_q = Quaterniond(Rs[frame_count - 1].inverse() * Rs[frame_count]);

                    if (initial_ex_rotation.CalibrationExRotation(corres, delta_q, calib_ric)) {
                        ROS_WARN("initial extrinsic rotation calibration success");
                        ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                        ric[0] = calib_ric;
                        RIC[0] = calib_ric;
                        ESTIMATE_EXTRINSIC = 1;
                    }
                }
            }
        }

        if (solver_flag == INITIAL) {
            // monocular + IMU initialization
            if (!STEREO && USE_IMU) {
                // frame_count is the number of frames in the sliding window
                if (frame_count == WINDOW_SIZE) {
                    bool result = false;
                    if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1) {
                        result = initialStructure();
                        initial_timestamp = header;
                    }
                    if (result) {
                        optimization();
//                        updateLatestStates();
                        solver_flag = NON_LINEAR;
                        slideWindow();
                        ROS_INFO("Initialization finish!");
                    } else
                        slideWindow();
                }
            }
                // stereo + IMU initialization
            else if (STEREO && USE_IMU) {
                f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
                f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
                if (frame_count == WINDOW_SIZE) {
                    map<double, ImageFrame>::iterator frame_it;
                    int i = 0;
                    for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++) {
                        frame_it->second.R = Rs[i];
                        frame_it->second.T = Ps[i];
                        i++;
                    }
                    solveGyroscopeBias(all_image_frame, Bgs);
                    for (int ii = 0; ii <= WINDOW_SIZE; ii++) {
                        pre_integrations[ii]->repropagate(Vector3d::Zero(), Bgs[ii]);
                    }
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
            }
                // stereo + INSPVA initialization
            else if (STEREO && !USE_IMU) {
                f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
                f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
                optimization();
                if (frame_count == WINDOW_SIZE) {
        //             optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    slideWindow();
                    ROS_INFO("Initialization finish!");
                }
            }

            if (frame_count < WINDOW_SIZE) {
                frame_count++;
                int prev_frame = frame_count - 1;
                Ps[frame_count] = Ps[prev_frame];
                Vs[frame_count] = Vs[prev_frame];
                Rs[frame_count] = Rs[prev_frame];
                Bas[frame_count] = Bas[prev_frame];
                Bgs[frame_count] = Bgs[prev_frame];
            }
        } else {
#ifdef SHOW_PROFILING
            utility::Timer t_solve;
            t_solve.start();
#endif // SHOW_PROFILING

            if (!USE_IMU) {
                 if(f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric)) {
                     f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
                 } else {
                     solver_flag = INITIAL;
                     return;
                 }
            } else
                f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

            optimization();

            set<int> removeIndex;
            outliersRejection(removeIndex);
            f_manager.removeOutlier(removeIndex);

            if (!MULTIPLE_THREAD) {
                featureTracker.removeOutliers(removeIndex);
                predictPtsInNextFrame();
            }
#ifdef SHOW_PROFILING
	        t_solve.stop();
            WriteToLog("    solver costs: ", t_solve);
#endif // SHOW_PROFILING

            if (frame_count > WINDOW_SIZE) {
                if (failureDetection()) {
                    ROS_WARN("failure detection!");
                    failure_occur = true;
                    clearState();
                    setParameter();
                    ROS_WARN("system reboot!");
                    return;
                }
            }
            slideWindow();

            f_manager.removeFailures();
            // prepare output of sslam
            key_poses.clear();
            for (int i = 0; i <= WINDOW_SIZE; i++)
                key_poses.push_back(Ps[i]);

            last_R = Rs[WINDOW_SIZE];
            last_P = Ps[WINDOW_SIZE];
            last_R0 = Rs[0];
            last_P0 = Ps[0];
            updateLatestStates();
        }
    }

    bool Estimator::initialStructure() {
        //check imu observability
        {
            map<double, ImageFrame>::iterator frame_it;
            Vector3d sum_g;
            for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
                double dt = frame_it->second.pre_integration->sum_dt;
                Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
                sum_g += tmp_g;
            }
            Vector3d aver_g;
            aver_g = sum_g * 1.0 / ((int) all_image_frame.size() - 1);
            double var = 0;
            for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
                double dt = frame_it->second.pre_integration->sum_dt;
                Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
                var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
                //cout << "frame g " << tmp_g.transpose() << endl;
            }
            var = sqrt(var / ((int) all_image_frame.size() - 1));
            //ROS_WARN("IMU variation %f!", var);
            if (var < 0.25) {
                ROS_INFO("IMU excitation not enough!");
                //return false;
            }
        }
        // global sfm
        Quaterniond Q[frame_count + 1];
        Vector3d T[frame_count + 1];
        map<int, Vector3d> sfm_tracked_points;
        vector<SFMFeature> sfm_f;
        for (auto &it_per_id : f_manager.feature) {
            int imu_j = it_per_id.start_frame - 1;
            SFMFeature tmp_feature;
            tmp_feature.state = false;
            tmp_feature.id = it_per_id.feature_id;
            for (auto &it_per_frame : it_per_id.feature_per_frame) {
                imu_j++;
                Vector3d pts_j = it_per_frame.point;
                tmp_feature.observation.emplace_back(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()});
            }
            sfm_f.push_back(tmp_feature);
        }
        Matrix3d relative_R;
        Vector3d relative_T;
        int l;
        if (!relativePose(relative_R, relative_T, l)) {
            ROS_INFO("Not enough features or parallax; Move device around");
            return false;
        }
        GlobalSFM sfm;
        if (!sfm.construct(frame_count + 1, Q, T, l,
                           relative_R, relative_T,
                           sfm_f, sfm_tracked_points)) {
#ifdef SHOW_PROFILING
	        Logger::Write("            [Fail] Global SFM failed!\n");
#endif // SHOW_PROFILING
            marginalization_flag = MARGIN_OLD;
            return false;
        }

        //solve pnp for all frame
        map<double, ImageFrame>::iterator frame_it;
        map<int, Vector3d>::iterator it;
        frame_it = all_image_frame.begin();
        for (int i = 0; frame_it != all_image_frame.end(); frame_it++) {
            // provide initial guess
            cv::Mat r, rvec, t, D, tmp_r;
            if ((frame_it->first) == Headers[i]) {
                frame_it->second.is_key_frame = true;
                frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
                frame_it->second.T = T[i];
                i++;
                continue;
            }
            if ((frame_it->first) > Headers[i]) {
                i++;
            }
            Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
            Vector3d P_inital = -R_inital * T[i];
            cv::eigen2cv(R_inital, tmp_r);
            cv::Rodrigues(tmp_r, rvec);
            cv::eigen2cv(P_inital, t);

            frame_it->second.is_key_frame = false;
            vector<cv::Point3f> pts_3_vector;
            vector<cv::Point2f> pts_2_vector;
            for (auto &id_pts : frame_it->second.points) {
                int feature_id = id_pts.first;
                for (auto &i_p : id_pts.second) {
                    it = sfm_tracked_points.find(feature_id);
                    if (it != sfm_tracked_points.end()) {
                        Vector3d world_pts = it->second;
                        cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                        pts_3_vector.push_back(pts_3);
                        Vector2d img_pts = i_p.second.head<2>();
                        cv::Point2f pts_2(img_pts(0), img_pts(1));
                        pts_2_vector.push_back(pts_2);
                    }
                }
            }
            cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
            if (pts_3_vector.size() < 6) {
                cout << "pts_3_vector size " << pts_3_vector.size() << endl;
#ifdef SHOW_PROFILING
	            Logger::Write("            [Fail] Not enough points for solve pnp !\n");
#endif // SHOW_PROFILING

                return false;
            }
            if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
#ifdef SHOW_PROFILING
	            Logger::Write("            [Fail] Solve pnp fail!\n");
#endif // SHOW_PROFILING
                return false;
            }
            cv::Rodrigues(rvec, r);
            MatrixXd R_pnp, tmp_R_pnp;
            cv::cv2eigen(r, tmp_R_pnp);
            R_pnp = tmp_R_pnp.transpose();
            MatrixXd T_pnp;
            cv::cv2eigen(t, T_pnp);
            T_pnp = R_pnp * (-T_pnp);
            frame_it->second.R = R_pnp * RIC[0].transpose();
            frame_it->second.T = T_pnp;
        }
        if (visualInitialAlign())
            return true;
        else {
            ROS_INFO("mis align visual structure with IMU");
            return false;
        }
    }

    bool Estimator::visualInitialAlign() {
        VectorXd x;
        //solve scale
        bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
        if (!result) {
#ifdef SHOW_PROFILING
	        Logger::Write("            [Fail] Solve gravity G failed!\n");
#endif // SHOW_PROFILING
            return false;
        }

        // change state
        for (int i = 0; i <= frame_count; i++) {
            Matrix3d Ri = all_image_frame[Headers[i]].R;
            Vector3d Pi = all_image_frame[Headers[i]].T;
            Ps[i] = Pi;
            Rs[i] = Ri;
            all_image_frame[Headers[i]].is_key_frame = true;
        }

        double s = (x.tail<1>())(0);
        for (int i = 0; i <= WINDOW_SIZE; i++) {
            pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
        }
        for (int i = frame_count; i >= 0; i--)
            Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
        int kv = -1;
        map<double, ImageFrame>::iterator frame_i;
        for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++) {
            if (frame_i->second.is_key_frame) {
                kv++;
                Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
            }
        }

        Matrix3d R0 = Utility::g2R(g);
        double yaw = Utility::R2ypr(R0 * Rs[0]).x();
        R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
        g = R0 * g;
        //Matrix3d rot_diff = R0 * Rs[0].transpose();
        Matrix3d rot_diff = R0;
        for (int i = 0; i <= frame_count; i++) {
            Ps[i] = rot_diff * Ps[i];
            Rs[i] = rot_diff * Rs[i];
            Vs[i] = rot_diff * Vs[i];
        }
//        ROS_DEBUG_STREAM("g0     " << g.transpose());
//        ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

        f_manager.clearDepth();
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

        return true;
    }

    bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l) {
        // find previous frame which contains enough correspondences and parallax with newest frame
        for (int i = 0; i < WINDOW_SIZE; i++) {
            vector<pair<Vector3d, Vector3d>> corres;
            corres = f_manager.getCorresponding(i, WINDOW_SIZE);
            if (corres.size() > 20) {
                double sum_parallax = 0;
                double average_parallax;
                for (auto &corre : corres) {
                    Vector2d pts_0(corre.first(0), corre.first(1));
                    Vector2d pts_1(corre.second(0), corre.second(1));
                    double parallax = (pts_0 - pts_1).norm();
                    sum_parallax = sum_parallax + parallax;

                }
                average_parallax = 1.0 * sum_parallax / int(corres.size());
                if (average_parallax * FOCAL_LENGTH > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
                    l = i;
#ifdef SHOW_PROFILING
	                Logger::Write("        Average_parallax " +
	                std::to_string(average_parallax * FOCAL_LENGTH) +
	                " choose l " + std::to_string(l) + " and newest frame to triangulate the whole structure\n");
#endif // SHOW_PROFILING
                    return true;
                }
            }
        }
        return false;
    }

    void Estimator::vector2double() {
        for (int i = 0; i <= WINDOW_SIZE; i++) {
            para_Pose[i][0] = Ps[i].x();
            para_Pose[i][1] = Ps[i].y();
            para_Pose[i][2] = Ps[i].z();
            Quaterniond q{Rs[i]};
            para_Pose[i][3] = q.x();
            para_Pose[i][4] = q.y();
            para_Pose[i][5] = q.z();
            para_Pose[i][6] = q.w();

            if (USE_IMU) {
                para_SpeedBias[i][0] = Vs[i].x();
                para_SpeedBias[i][1] = Vs[i].y();
                para_SpeedBias[i][2] = Vs[i].z();

                para_SpeedBias[i][3] = Bas[i].x();
                para_SpeedBias[i][4] = Bas[i].y();
                para_SpeedBias[i][5] = Bas[i].z();

                para_SpeedBias[i][6] = Bgs[i].x();
                para_SpeedBias[i][7] = Bgs[i].y();
                para_SpeedBias[i][8] = Bgs[i].z();
            }
        }

        for (int i = 0; i < NUM_OF_CAM; i++) {
            para_Ex_Pose[i][0] = tic[i].x();
            para_Ex_Pose[i][1] = tic[i].y();
            para_Ex_Pose[i][2] = tic[i].z();
            Quaterniond q{ric[i]};
            para_Ex_Pose[i][3] = q.x();
            para_Ex_Pose[i][4] = q.y();
            para_Ex_Pose[i][5] = q.z();
            para_Ex_Pose[i][6] = q.w();
        }


        VectorXd dep = f_manager.getDepthVector();
        for (int i = 0; i < f_manager.getFeatureCount(); i++)
            para_Feature[i][0] = dep(i);

        para_Td[0][0] = td;
    }

    void Estimator::double2vector() {
        Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
        Vector3d origin_P0 = Ps[0];

        if (failure_occur) {
            origin_R0 = Utility::R2ypr(last_R0);
            origin_P0 = last_P0;
            failure_occur = false;
        }

        if (USE_IMU) {
            Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                             para_Pose[0][3],
                                                             para_Pose[0][4],
                                                             para_Pose[0][5]).toRotationMatrix());
            double y_diff = origin_R0.x() - origin_R00.x();
            Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
            if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
#ifdef SHOW_PROFILING
	            Logger::Write("#                    Euler singular point!\n");
#endif // SHOW_PROFILING
                rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                               para_Pose[0][3],
                                               para_Pose[0][4],
                                               para_Pose[0][5]).toRotationMatrix().transpose();
            }

            for (int i = 0; i <= WINDOW_SIZE; i++) {

                Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4],
                                               para_Pose[i][5]).normalized().toRotationMatrix();

                Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                            para_Pose[i][1] - para_Pose[0][1],
                                            para_Pose[i][2] - para_Pose[0][2]) + origin_P0;

	            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
	                                        para_SpeedBias[i][1],
	                                        para_SpeedBias[i][2]);

	            Bas[i] = Vector3d(para_SpeedBias[i][3],
	                              para_SpeedBias[i][4],
	                              para_SpeedBias[i][5]);

	            Bgs[i] = Vector3d(para_SpeedBias[i][6],
	                              para_SpeedBias[i][7],
	                              para_SpeedBias[i][8]);

            }
        } else {
            for (int i = 0; i <= WINDOW_SIZE; i++) {
                Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4],
                                    para_Pose[i][5]).normalized().toRotationMatrix();

                Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
            }
        }

//	    Vector3d T_;
//	    Matrix3d R_;
//
//        if( !init_kalman && solver_flag == NON_LINEAR) {
//        	kalman_.init(curTime, Ps[WINDOW_SIZE], Rs[WINDOW_SIZE]);
//        	init_kalman = true;
//        } else if(init_kalman) {
//	        kalman_.update(curTime, Ps[WINDOW_SIZE], Rs[WINDOW_SIZE]);
//	        kalman_.getPoseState(T_, R_);
//	        Ps[WINDOW_SIZE] = T_;
//	        Rs[WINDOW_SIZE] = R_;
//        }

        if (USE_IMU || USE_INS) {
            for (int i = 0; i < NUM_OF_CAM; i++) {
                tic[i] = Vector3d(para_Ex_Pose[i][0],
                                  para_Ex_Pose[i][1],
                                  para_Ex_Pose[i][2]);
                ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                     para_Ex_Pose[i][3],
                                     para_Ex_Pose[i][4],
                                     para_Ex_Pose[i][5]).normalized().toRotationMatrix();
            }
        }

        VectorXd dep = f_manager.getDepthVector();
        for (int i = 0; i < f_manager.getFeatureCount(); i++)
            dep(i) = para_Feature[i][0];
        f_manager.setDepth(dep);

        if (USE_IMU || USE_INS)
            td = para_Td[0][0];
    }

    bool Estimator::failureDetection() {
//    return false;
        if (f_manager.last_track_num < 2) {
            ROS_INFO(" little feature %d", f_manager.last_track_num);
            //return true;
        }
        if (Bas[WINDOW_SIZE].norm() > 2.5) {
            ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
            return true;
        }
        if (Bgs[WINDOW_SIZE].norm() > 1.0) {
            ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
            return true;
        }
        /*
        if (tic(0) > 1)
        {
            ROS_INFO(" big extri param estimation %d", tic(0) > 1);
            return true;
        }
        */
        Vector3d tmp_P = Ps[WINDOW_SIZE];
        if ((tmp_P - last_P).norm() > 6) {
            ROS_INFO(" big translation");
            return true;
        }
        if (abs(tmp_P.z() - last_P.z()) > 3) {
            ROS_INFO(" big z translation");
            return true;
        }
        Matrix3d tmp_R = Rs[WINDOW_SIZE];
        Matrix3d delta_R = tmp_R.transpose() * last_R;
        Quaterniond delta_Q(delta_R);
        double delta_angle;
        delta_angle = acos(delta_Q.w());
        if (delta_angle > 0.4363) {
            ROS_INFO(" big delta_angle ");
            return true;
        }
        return false;
    }

    void Estimator::optimization() {
#ifdef SHOW_PROFILING
	    utility::Timer t_whole;
	    t_whole.start();
        utility::Timer t_solver;
        if(count_ % 20 == 0 && solver_flag == NON_LINEAR) {
            t_solver.start();
        }
#endif // SHOW_PROFILING

        vector2double();

        ceres::Problem problem;
        ceres::LossFunction *loss_function;
        loss_function = new ceres::HuberLoss(1.0);
//    auto* ordering = new ceres::ParameterBlockOrdering;
//    shared_ptr<ceres::ParameterBlockOrdering> ordering;
//        loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
        for (int i = 0; i < frame_count + 1; i++) {
            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
            if (USE_IMU)
                problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
        }
        if (!USE_IMU)
            problem.SetParameterBlockConstant(para_Pose[0]);

        for (int i = 0; i < NUM_OF_CAM; i++) {
            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
            if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE) || openExEstimation) {
                //estimate extrinsic param
                openExEstimation = true;
            } else {
                //fix extrinsic param
                problem.SetParameterBlockConstant(para_Ex_Pose[i]);
            }
        }
        problem.AddParameterBlock(para_Td[0], 1);

        if (!ESTIMATE_TD)
            problem.SetParameterBlockConstant(para_Td[0]);

        if (last_marginalization_info && last_marginalization_info->valid) {
            // construct new marginalization_factor
            auto *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            problem.AddResidualBlock(marginalization_factor, nullptr,
                                     last_marginalization_parameter_blocks);
        }
        if (USE_IMU) {
            for (int i = 0; i < frame_count; i++) {
                int j = i + 1;
                if (pre_integrations[j]->sum_dt > 10.0)
                    continue;
                auto *imu_factor = new IMUFactor(pre_integrations[j]);
                problem.AddResidualBlock(imu_factor, nullptr, para_Pose[i],
                        para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
            }
        } else if (USE_INS && !USE_GPS) {
            for (int i = 0; i < frame_count; i++) {
                int j = i + 1;
                if (sum_dt[j] > 10.0)
                    continue;
                Eigen::Vector3d delta_P = Eigen::Vector3d().setZero();
                for (size_t kk = 0; kk < dt_buf[j].size(); kk++) {

                        double t_ = dt_buf[j].at(kk);
                        delta_P += linear_speed_buf[j].at(kk) * t_;
                }

                Eigen::Quaterniond ang_read = angular_read_buf[j].back();
//	            double height_read_delta = height_read_buf[j].back() - height_read_buf[i].back();
//	            cout << "height delta1: " << height_read_delta << " | 2: " << delta_P.z() << endl;
	                 ceres::CostFunction *ins_factor = INSRTError::Create(delta_P.x(), delta_P.y(), delta_P.z(),
                                                                     ang_read.w(), ang_read.x(), ang_read.y(),
                                                                     ang_read.z(), 0.1, 0.01);
                problem.AddResidualBlock(ins_factor, loss_function, para_Pose[i], para_Pose[j]);
//            ceres::CostFunction* ins_factor = INSRError::Create(ang_read.w(), ang_read.x(), ang_read.y(),
//                                                                ang_read.z(), 0.01);
//            problem.AddResidualBlock(ins_factor, loss_function, para_Pose[j]);
            }
        } else if (USE_INS) {
            gps_bad = false;
            for (int i = 0; i < frame_count + 1; i++) {
                gps_bad = gps_bad | (bool)gps_status[i].back();
            }
//            cout << "gps_bad: " << gps_bad << endl;

            for (int i = 0; i < frame_count; i++) {
                double cov_gps1 = gps_buf[i+1].back()[3];
                double cov_gps2 = gps_buf[i+1].back()[4];

                if(!gps_bad) {
                    Vector5d delta_P = gps_buf[i+1].back() - gps_buf[i].back();

                    Eigen::Quaterniond ang_read = angular_read_buf[i+1].back();
//	                double height_read_delta = height_read_buf[i+1].back() - height_read_buf[i].back();
                    ceres::CostFunction *ins_factor = RelativeRTError::Create(delta_P[0], delta_P[1], delta_P[2],
                                                                         ang_read.w(), ang_read.x(), ang_read.y(),
                                                                         ang_read.z(), cov_gps1, cov_gps2, 0.01);
//                    ceres::CostFunction *ins_factor = TError::Create(delta_P[0], delta_P[1], delta_P[2],
//                                                                            ang_read.w(), ang_read.x(), ang_read.y(),
//                                                                         ang_read.z(), cov_gps1, cov_gps2);
                    problem.AddResidualBlock(ins_factor, loss_function, para_Pose[i], para_Pose[i+1]);
                }
            }
        }

        int f_m_cnt = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature) {
            it_per_id.used_num = static_cast<int>(it_per_id.feature_per_frame.size());
            if (it_per_id.used_num < 4)
                continue;

            ++feature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            for (auto &it_per_frame : it_per_id.feature_per_frame) {
                imu_j++;

                if (imu_i != imu_j) {
                    Vector3d pts_j = it_per_frame.point;
                    auto *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j,
                                                                    it_per_id.feature_per_frame[0].velocity,
                                                                    it_per_frame.velocity,
                                                                    it_per_id.feature_per_frame[0].cur_td,
                                                                    it_per_frame.cur_td);
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i],
                    		para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                }

                if (STEREO && it_per_frame.is_stereo) {
                    Vector3d pts_j_right = it_per_frame.pointRight;
                    if (imu_i != imu_j) {
                        auto *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right,
                                                                     it_per_id.feature_per_frame[0].velocity,
                                                                     it_per_frame.velocityRight,
                                                                     it_per_id.feature_per_frame[0].cur_td,
                                                                     it_per_frame.cur_td);
                        problem.AddResidualBlock(f, loss_function, para_Pose[imu_i],
                        		para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                    } else {
                        auto *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right,
                                                                     it_per_id.feature_per_frame[0].velocity,
                                                                     it_per_frame.velocityRight,
                                                                     it_per_id.feature_per_frame[0].cur_td,
                                                                     it_per_frame.cur_td);
                        problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1],
                                                 para_Feature[feature_index], para_Td[0]);
                    }

                }
                f_m_cnt++;
            }
        }

//	    if(frame_count > 1) {
//		    for (int i = 1; i < frame_count; i++) {
//			    for (int j = 1; j < 5; j++) {
//				    if (i - j >= 0) {
//					    Eigen::Vector3d relative_t(para_Pose[i][0] - para_Pose[i - j][0],
//					                               para_Pose[i][1] - para_Pose[i - j][1],
//					                               para_Pose[i][2] - para_Pose[i - j][2]);
//
//					    Quaterniond q_i_j = Quaterniond(para_Pose[i - j][6], para_Pose[i - j][3],
//					                                    para_Pose[i - j][4],
//					                                    para_Pose[i - j][5]);
//					    Quaterniond q_i = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4],
//					                                  para_Pose[i][5]);
//					    relative_t = q_i_j.inverse() * relative_t;
//					    Quaterniond relative_q = q_i_j.inverse() * q_i;
//					    ceres::CostFunction *vo_function = RelativeRTError::Create(relative_t.x(),
//					                                                               relative_t.y(),
//					                                                               relative_t.z(),
//					                                                               relative_q.w(),
//					                                                               relative_q.x(),
//					                                                               relative_q.y(),
//					                                                               relative_q.z(),
//					                                                               0.1, 0.01);
//					    problem.AddResidualBlock(vo_function, nullptr, para_Pose[i - j], para_Pose[i]);
//				    }
//			    }
//		    }
//	    }

#ifdef SHOW_PROFILING
	    Logger::Write("   Visual measurement count:" + std::to_string(f_m_cnt) + "\n");
#endif // SHOW_PROFILING

        ceres::Solver::Options options;

        if(OUTPUT_COV == 0)
            options.linear_solver_type = ceres::DENSE_SCHUR;
        else
            options.linear_solver_type = ceres::SPARSE_SCHUR;
//        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
//        options.use_explicit_schur_complement = true;
        options.num_threads = 6;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = NUM_ITERATIONS;
        //options.minimizer_progress_to_stdout = true;
        //options.use_nonmonotonic_steps = true;
        if (marginalization_flag == MARGIN_OLD)
            options.max_solver_time_in_seconds = SOLVER_TIME * 0.8;
        else
            options.max_solver_time_in_seconds = SOLVER_TIME;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // Covariance Estimation!
    if(count_ % 20 == 0 && solver_flag == NON_LINEAR && OUTPUT_COV == 1) {
#ifdef SHOW_PROFILING
        utility::Timer t_cov;
        t_cov.start();
#endif // SHOW_PROFILING
//        cout << summary.BriefReport() << endl;
//        cout << summary.FullReport() << endl;
#ifdef SHOW_PROFILING
	    t_solver.stop();
	    Logger::Write("    Optimization solver iterations " + std::to_string(summary.iterations.size()) + "\n");
	    WriteToLog("  Optimization solver cost time  ", t_solver);
#endif // SHOW_PROFILING
//
//    cout << "Total cost time: " << summary.total_time_in_seconds <<
//    " | linear solver: " << summary.linear_solver_time_in_seconds <<
//    " | TrustRegionMinimizer time: " << summary.minimizer_time_in_seconds << endl;
//
//        // Covariance of poses
        ceres::Covariance::Options cov_options;
        cov_options.num_threads = 6;
        ceres::Covariance covariance(cov_options);
//
        std::vector<std::pair<const double *, const double *>> covariance_blocks;
        covariance_blocks.emplace_back(para_Pose[WINDOW_SIZE], para_Pose[WINDOW_SIZE]);
////        CHECK(covariance.Compute(covariance_blocks, &problem));
        if(covariance.Compute(covariance_blocks, &problem)) {
            double covariance_pose[SIZE_POSE * SIZE_POSE];
            covariance.GetCovarianceBlock(para_Pose[WINDOW_SIZE], para_Pose[WINDOW_SIZE], covariance_pose);

#ifdef SHOW_PROFILING
	    t_cov.stop();
	    WriteToLog("Covariance solver costs  ", t_cov);
#endif // SHOW_PROFILING

            cov_position(0, 0) = covariance_pose[0];
            cov_position(0, 1) = covariance_pose[1];
            cov_position(0, 2) = covariance_pose[2];
            cov_position(1, 0) = covariance_pose[7];
            cov_position(1, 1) = covariance_pose[8];
            cov_position(1, 2) = covariance_pose[9];
            cov_position(2, 0) = covariance_pose[14];
            cov_position(2, 1) = covariance_pose[15];
            cov_position(2, 2) = covariance_pose[16];
        }
    }

        count_++;

        double2vector();
        //printf("frame_count: %d \n", frame_count);

        if (frame_count < WINDOW_SIZE)
            return;

        // Consider marginalizing old keyframe and refreshing sliding window.

#ifdef SHOW_PROFILING
	    utility::Timer t_whole_marginalization;
	    t_whole_marginalization.start();
#endif // SHOW_PROFILING

	    // Marginalize the oldest frame in the sliding window.
        if (marginalization_flag == MARGIN_OLD) {
            auto *marginalization_info = new MarginalizationInfo();
            vector2double();

            if (last_marginalization_info && last_marginalization_info->valid) {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                    if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                        last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                        drop_set.push_back(i);
                }
                // construct new marginalization_factor
	            // 1. Pass the residual factors of last prior factor (size is n) to current prior factor.
	            auto *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                auto residual_block_info = new ResidualBlockInfo(marginalization_factor, nullptr,
                                                                 last_marginalization_parameter_blocks,
                                                                 drop_set);
                // Add both parameter blocks and drop sets(variables to be marginalized).
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            // 2. Pass the residual factors of current sliding window oldest IMU/INS/Camera observations.
            if (USE_IMU) {
                if (pre_integrations[1]->sum_dt < 10.0) {
                    auto *imu_factor = new IMUFactor(pre_integrations[1]);
                    auto residual_block_info = new ResidualBlockInfo(imu_factor, nullptr,
                                                                     vector<double *>{para_Pose[0], para_SpeedBias[0],
                                                                                      para_Pose[1], para_SpeedBias[1]},
                                                                     vector<int>{0, 1});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            } else if (USE_INS && !USE_GPS) {
                if (sum_dt[1] < 10.0) {
                    Eigen::Vector3d delta_P = Eigen::Vector3d().setZero();
                    for (size_t kk = 0; kk < dt_buf[1].size(); kk++) {
                            double t_ = dt_buf[1].at(kk);
                            delta_P += linear_speed_buf[1].at(kk) * t_;
                    }
                    Eigen::Quaterniond ang_read = angular_read_buf[1].back();
//	                double height_read_delta = height_read_buf[1].back() - height_read_buf[0].back();
                    ceres::CostFunction *ins_factor = INSRTError::Create(delta_P.x(), delta_P.y(), delta_P.z(),
                                                                         ang_read.w(), ang_read.x(), ang_read.y(),
                                                                         ang_read.z(), 0.01, 0.007);
                    auto residual_block_info = new ResidualBlockInfo(ins_factor, loss_function,
                                                                     vector<double *>{para_Pose[0], para_Pose[1]},
                                                                     vector<int>{0});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            } else if (USE_INS) {
	            double cov_gps1 = gps_buf[1].back()[3];
                double cov_gps2 = gps_buf[1].back()[4];
                bool gps_stat_1 = gps_status[0].back();
                bool gps_stat_2 = gps_status[1].back();
                if(!gps_stat_1 && !gps_stat_2 && !gps_bad) {
//	            bool gps_stat_ = gps_status[0].back();
//	            if(!gps_stat_) {
		            Vector5d delta_P = gps_buf[1].back() - gps_buf[0].back();

		            Eigen::Quaterniond ang_read = angular_read_buf[1].back();
//	                double height_read_delta = height_read_buf[1].back() - height_read_buf[0].back();
		            ceres::CostFunction *ins_factor = RelativeRTError::Create(delta_P[0], delta_P[1], delta_P[2],
		                                                                 ang_read.w(), ang_read.x(), ang_read.y(),
		                                                                 ang_read.z(), cov_gps1, cov_gps2, 0.01);
//                    ceres::CostFunction *ins_factor = TError::Create(delta_P[0], delta_P[1], delta_P[2],
//                                                                         ang_read.w(), ang_read.x(), ang_read.y(),
//		                                                                 ang_read.z(),cov_gps1, cov_gps2);
		            auto residual_block_info = new ResidualBlockInfo(ins_factor, loss_function,
		                                                             vector<double *>{para_Pose[0], para_Pose[1]},
		                                                             vector<int>{0});
		            marginalization_info->addResidualBlockInfo(residual_block_info);
	            }
            }

	        {
                int feature_index_local = -1;
                for (auto &it_per_id : f_manager.feature) {
                    it_per_id.used_num = it_per_id.feature_per_frame.size();
                    if (it_per_id.used_num < 4)
                        continue;

                    ++feature_index_local;

                    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                    if (imu_i != 0)
                        continue;

                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                    for (auto &it_per_frame : it_per_id.feature_per_frame) {
                        imu_j++;
                        if (imu_i != imu_j) {
                            Vector3d pts_j = it_per_frame.point;
                            auto *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j,
                                                                            it_per_id.feature_per_frame[0].velocity,
                                                                            it_per_frame.velocity,
                                                                            it_per_id.feature_per_frame[0].cur_td,
                                                                            it_per_frame.cur_td);
                            auto residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                             vector<double *>{para_Pose[imu_i],
                                                                                              para_Pose[imu_j],
                                                                                              para_Ex_Pose[0],
                                                                                              para_Feature[feature_index_local],
                                                                                              para_Td[0]},
                                                                             vector<int>{0, 3});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        if (STEREO && it_per_frame.is_stereo) {
                            Vector3d pts_j_right = it_per_frame.pointRight;
                            if (imu_i != imu_j) {
                                auto *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right,
                                                                             it_per_id.feature_per_frame[0].velocity,
                                                                             it_per_frame.velocityRight,
                                                                             it_per_id.feature_per_frame[0].cur_td,
                                                                             it_per_frame.cur_td);
                                auto residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                 vector<double *>{para_Pose[imu_i],
                                                                                                  para_Pose[imu_j],
                                                                                                  para_Ex_Pose[0],
                                                                                                  para_Ex_Pose[1],
                                                                                                  para_Feature[feature_index_local],
                                                                                                  para_Td[0]},
                                                                                 vector<int>{0, 4});
                                marginalization_info->addResidualBlockInfo(residual_block_info);
                            } else {
                                auto *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right,
                                                                             it_per_id.feature_per_frame[0].velocity,
                                                                             it_per_frame.velocityRight,
                                                                             it_per_id.feature_per_frame[0].cur_td,
                                                                             it_per_frame.cur_td);
                                auto residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                 vector<double *>{para_Ex_Pose[0],
                                                                                                  para_Ex_Pose[1],
                                                                                                  para_Feature[feature_index_local],
                                                                                                  para_Td[0]},
                                                                                 vector<int>{2});
                                marginalization_info->addResidualBlockInfo(residual_block_info);
                            }
                        }
                    }
                }
            }
#ifdef SHOW_PROFILING
	        utility::Timer t_pre_margin;
	        t_pre_margin.start();
#endif // SHOW_PROFILING

            marginalization_info->preMarginalize();

#ifdef SHOW_PROFILING
	        t_pre_margin.stop();
	        WriteToLog("  Pre marginalization  ", t_pre_margin);
	        utility::Timer t_margin;
	        t_margin.start();
#endif // SHOW_PROFILING

            marginalization_info->marginalize();

#ifdef SHOW_PROFILING
	        t_margin.stop();
	        WriteToLog("  Marginalization  ", t_margin);
#endif // SHOW_PROFILING

            std::unordered_map<long, double *> addr_shift;
            // As we abandoned the oldest frame, we shift the remaining frames one position backward.
            for (int i = 1; i <= WINDOW_SIZE; i++) {
                addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                if (USE_IMU)
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

            delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

        } else {
            if (last_marginalization_info &&
                std::count(std::begin(last_marginalization_parameter_blocks),
                           std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1])) {
                auto *marginalization_info = new MarginalizationInfo();
                vector2double();
                if (last_marginalization_info->valid) {
                    vector<int> drop_set;
                    for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                        ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                        if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                            drop_set.push_back(i);
                    }
                    // construct new marginalization_factor
                    auto *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                    auto residual_block_info = new ResidualBlockInfo(marginalization_factor, nullptr,
                                                                     last_marginalization_parameter_blocks,
                                                                     drop_set);

                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }


#ifdef SHOW_PROFILING
	            utility::Timer t_pre_margin;
	            t_pre_margin.start();
#endif // SHOW_PROFILING

	            marginalization_info->preMarginalize();

#ifdef SHOW_PROFILING
	            t_pre_margin.stop();
	            WriteToLog("  Pre marginalization  ", t_pre_margin);
	            utility::Timer t_margin;
	            t_margin.start();
#endif // SHOW_PROFILING

	            marginalization_info->marginalize();

#ifdef SHOW_PROFILING
	            t_margin.stop();
	            WriteToLog("  Marginalization  ", t_margin);
#endif // SHOW_PROFILING

                std::unordered_map<long, double *> addr_shift;
                for (int i = 0; i <= WINDOW_SIZE; i++) {
                    if (i == WINDOW_SIZE - 1)
                        continue;
                    else if (i == WINDOW_SIZE) {
                        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                        if (USE_IMU)
                            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                    } else {
                        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                        if (USE_IMU)
                            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                    }
                }
                for (int i = 0; i < NUM_OF_CAM; i++)
                    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

                vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
                delete last_marginalization_info;
                last_marginalization_info = marginalization_info;
                last_marginalization_parameter_blocks = parameter_blocks;

            }
        }
#ifdef SHOW_PROFILING
		t_whole.stop();
	    t_whole_marginalization.stop();
	    WriteToLog(" Whole marginalization  ", t_whole_marginalization);
	    WriteToLog(" Whole time for ceres  ", t_whole);
#endif // SHOW_PROFILING
    }

    void Estimator::slideWindow() {
        if (marginalization_flag == MARGIN_OLD) {
            double t_0 = Headers[0];
            back_R0 = Rs[0];
            back_P0 = Ps[0];
            if (frame_count == WINDOW_SIZE) {
                for (int i = 0; i < WINDOW_SIZE; i++) {
                    Headers[i] = Headers[i + 1];
                    Rs[i].swap(Rs[i + 1]);
                    Ps[i].swap(Ps[i + 1]);
                    if (USE_IMU) {
                        std::swap(pre_integrations[i], pre_integrations[i + 1]);

                        dt_buf[i].swap(dt_buf[i + 1]);
                        linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                        angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                        Vs[i].swap(Vs[i + 1]);
                        Bas[i].swap(Bas[i + 1]);
                        Bgs[i].swap(Bgs[i + 1]);
                    } else if (USE_INS) {
                        dt_buf[i].swap(dt_buf[i + 1]);
                        sum_dt[i] = sum_dt[i + 1];
                        angular_read_buf[i].swap(angular_read_buf[i + 1]);
//	                    height_read_buf[i].swap(height_read_buf[i + 1]);
                        Vs[i].swap(Vs[i + 1]);
                        if(!USE_GPS) {
	                        linear_speed_buf[i].swap(linear_speed_buf[i + 1]);
	                        t_buf[i].swap(t_buf[i + 1]);
                        } else {
	                        gps_buf[i].swap(gps_buf[i + 1]);
	                        gt_buf[i].swap(gt_buf[i + 1]);
	                        gps_status[i].swap(gps_status[i + 1]);
                        }
                    }

                }
                Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
                Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
                Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

                if (USE_IMU) {
                    Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                    Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                    Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                    delete pre_integrations[WINDOW_SIZE];
                    pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                                        Bgs[WINDOW_SIZE]};

                    dt_buf[WINDOW_SIZE].clear();
                    linear_acceleration_buf[WINDOW_SIZE].clear();
                    angular_velocity_buf[WINDOW_SIZE].clear();
                } else if (USE_INS) {
                    Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                    dt_buf[WINDOW_SIZE].clear();
                    sum_dt[WINDOW_SIZE] = 0;
                    angular_read_buf[WINDOW_SIZE].clear();
//	                height_read_buf[WINDOW_SIZE].clear();
                    if(USE_GPS) {
                        gt_buf[WINDOW_SIZE].clear();
                        gps_buf[WINDOW_SIZE].clear();
                        gps_status[WINDOW_SIZE].clear();
                    } else {
                        t_buf[WINDOW_SIZE].clear();
                        linear_speed_buf[WINDOW_SIZE].clear();
                    }
                }

                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
//                delete it_0->second.pre_integration;
	            if (it_0->second.pre_integration != nullptr) {
		            delete it_0->second.pre_integration;
		            it_0->second.pre_integration = nullptr;
	            }
                all_image_frame.erase(all_image_frame.begin(), it_0);
                slideWindowOld();
            }
        } else {
            if (frame_count == WINDOW_SIZE) {
                Headers[frame_count - 1] = Headers[frame_count];
                Ps[frame_count - 1] = Ps[frame_count];
                Rs[frame_count - 1] = Rs[frame_count];

                if (USE_IMU) {
                    for (size_t i = 0; i < dt_buf[frame_count].size(); i++) {
                        double tmp_dt = dt_buf[frame_count][i];
                        Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                        Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                        pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration,
                                                                     tmp_angular_velocity);

                        dt_buf[frame_count - 1].push_back(tmp_dt);
                        linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                        angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                    }

                    Vs[frame_count - 1] = Vs[frame_count];
                    Bas[frame_count - 1] = Bas[frame_count];
                    Bgs[frame_count - 1] = Bgs[frame_count];

                    delete pre_integrations[WINDOW_SIZE];
                    pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE],
                                                                        Bgs[WINDOW_SIZE]};

                    dt_buf[WINDOW_SIZE].clear();
                    linear_acceleration_buf[WINDOW_SIZE].clear();
                    angular_velocity_buf[WINDOW_SIZE].clear();
                } else if (USE_INS) {
                    for (size_t i = 0; i < dt_buf[frame_count].size(); i++) {
                        double tmp_dt = dt_buf[frame_count][i];
                        Quaterniond tmp_angular_read = angular_read_buf[frame_count][i];
                        dt_buf[frame_count - 1].push_back(tmp_dt);
                        angular_read_buf[frame_count - 1].push_back(tmp_angular_read);
                        if(!USE_GPS) {
	                        Vector3d tmp_linear_speed = linear_speed_buf[frame_count][i];
	                        linear_speed_buf[frame_count - 1].push_back(tmp_linear_speed);
                        } else {
	                        Vector5d tmp_gps = gps_buf[frame_count][i];
	                        gps_buf[frame_count - 1].emplace_back(tmp_gps);
	                        bool tmp_gps_status = gps_status[frame_count][i];
	                        gps_status[frame_count - 1].push_back(tmp_gps_status);
                        }
                    }

                    Vs[frame_count - 1] = Vs[frame_count];
                    dt_buf[WINDOW_SIZE].clear();
                    sum_dt[WINDOW_SIZE] = 0;
                    angular_read_buf[WINDOW_SIZE].clear();

                    if(USE_GPS) {
                        gps_buf[WINDOW_SIZE].clear();
	                    gps_status[WINDOW_SIZE].clear();
                        gt_buf[WINDOW_SIZE].clear();
                    } else {
                        linear_speed_buf[WINDOW_SIZE].clear();
//	                    height_read_buf[WINDOW_SIZE].clear();
	                    t_buf[WINDOW_SIZE].clear();
                    }
                }

                slideWindowNew();
            }
        }
    }

    void Estimator::slideWindowNew() {
        sum_of_front++;
        f_manager.removeFront(frame_count);
    }

    void Estimator::slideWindowOld() {
        sum_of_back++;

        bool shift_depth = solver_flag == NON_LINEAR;
        if (shift_depth) {
            Matrix3d R0, R1;
            Vector3d P0, P1;
            R0 = back_R0 * ric[0];
            R1 = Rs[0] * ric[0];
            P0 = back_P0 + back_R0 * tic[0];
            P1 = Ps[0] + Rs[0] * tic[0];
            f_manager.removeBackShiftDepth(R0, P0, R1, P1);
        } else
            f_manager.removeBack();
    }


    void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
        T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = Rs[frame_count];
        T.block<3, 1>(0, 3) = Ps[frame_count];
    }

    void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
        T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = Rs[index];
        T.block<3, 1>(0, 3) = Ps[index];
    }

    void Estimator::predictPtsInNextFrame() {
        //printf("predict pts in next frame\n");
        if (frame_count < 2)
            return;
        // predict next pose. Assume constant velocity motion
        Eigen::Matrix4d curT, prevT, nextT;
        getPoseInWorldFrame(curT);
        getPoseInWorldFrame(frame_count - 1, prevT);
        nextT = curT * (prevT.inverse() * curT);
        map<int, Eigen::Vector3d> predictPts;

        for (auto &it_per_id : f_manager.feature) {
            if (it_per_id.estimated_depth > 0) {
                int firstIndex = it_per_id.start_frame;
                int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
                //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
                if ((int) it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count) {
                    double depth = it_per_id.estimated_depth;
                    Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                    Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                    Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                    Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                    int ptsIndex = it_per_id.feature_id;
                    predictPts[ptsIndex] = pts_cam;
                }
            }
        }
        featureTracker.setPrediction(predictPts);
        //printf("estimator output %d predict pts\n",(int)predictPts.size());
    }

    double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                        Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                        double depth, Vector3d &uvi, Vector3d &uvj) {
        Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
        Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
        Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
        double rx = residual.x();
        double ry = residual.y();
        return sqrt(rx * rx + ry * ry);
    }

    void Estimator::outliersRejection(set<int> &removeIndex) {
        //return;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature) {
            double err = 0;
            int errCnt = 0;
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;
            feature_index++;
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;
            double depth = it_per_id.estimated_depth;
            for (auto &it_per_frame : it_per_id.feature_per_frame) {
                imu_j++;
                if (imu_i != imu_j) {
                    Vector3d pts_j = it_per_frame.point;
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                         depth, pts_i, pts_j);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                // need to rewrite projection factor.........
                if (STEREO && it_per_frame.is_stereo) {

                    Vector3d pts_j_right = it_per_frame.pointRight;
                    if (imu_i != imu_j) {
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                             Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                             depth, pts_i, pts_j_right);
                        err += tmp_error;
                        errCnt++;
                        //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                    } else {
                        double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                             Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                             depth, pts_i, pts_j_right);
                        err += tmp_error;
                        errCnt++;
                        //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                    }
                }
            }
            double ave_err = err / errCnt;
            if (ave_err * FOCAL_LENGTH > 3)
                removeIndex.insert(it_per_id.feature_id);

        }
    }

    void Estimator::fastPredictIMU(double t, const Eigen::Vector3d &linear_acceleration,
                                   const Eigen::Vector3d &angular_velocity) {
        double dt = t - latest_time;
        latest_time = t;
        Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
        Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
        latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
        Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
        latest_V = latest_V + dt * un_acc;
        latest_acc_0 = linear_acceleration;
        latest_gyr_0 = angular_velocity;
    }

    void Estimator::fastPredictINS(double t, const Eigen::Vector3d &linear_speed,
                                   const Eigen::Quaterniond &angular_read) {
        double dt = t - latest_time;
        latest_time = t;

        latest_V = last_vec_rev + last_ang_rev * linear_speed;

        latest_P = latest_P + latest_Q * (latest_V * dt);

        latest_Q = latest_Q * (last_ang_rev * angular_read);

        last_ang_rev = latest_Q.inverse();
        last_vec_rev = -last_ang_rev.toRotationMatrix() * latest_V;

    }

    void Estimator::updateLatestStates() {
        mPropagate.lock();

        latest_time = Headers[frame_count] + td;
        latest_P = Ps[frame_count];
        latest_Q = Rs[frame_count];
        latest_V = Vs[frame_count];
        latest_Ba = Bas[frame_count];
        latest_Bg = Bgs[frame_count];
        latest_acc_0 = acc_0;
        latest_gyr_0 = gyr_0;
//    latest_spd_0 = spd_0;
//    latest_ang_0 = ang_0;
        mBuf.lock();
        queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
        queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
        mBuf.unlock();
        while (!tmp_accBuf.empty()) {
            double t = tmp_accBuf.front().first;
            Eigen::Vector3d acc = tmp_accBuf.front().second;
            Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
            fastPredictIMU(t, acc, gyr);
            tmp_accBuf.pop();
            tmp_gyrBuf.pop();
        }
        queue<pair<double, Eigen::Vector3d>> tmp_spdBuf = spdBuf;
        queue<pair<double, Eigen::Quaterniond>> tmp_angBuf = angBuf;
//        while (!tmp_spdBuf.empty()) {
//            double t = tmp_spdBuf.front().first;
//            Eigen::Vector3d spd = tmp_spdBuf.front().second;
//            Eigen::Quaterniond ang = tmp_angBuf.front().second;
////        fastPredictINS(t, spd, ang);
//            tmp_spdBuf.pop();
//            tmp_angBuf.pop();
//        }
        mPropagate.unlock();
    }
}