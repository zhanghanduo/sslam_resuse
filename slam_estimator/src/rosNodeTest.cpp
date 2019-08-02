/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file rosNodeTest.cpp
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief The entry file (main function) of SSLAM-slam_estimator.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// Obstacle ros msgs
#include <obstacle_msgs/MapInfo.h>
#include <obstacle_msgs/obs.h>
#include <obstacle_msgs/point3.h>
#include <rds_msgs/msg_novatel_inspva.h>

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<obstacle_msgs::MapInfoConstPtr> dy_buf;
std::mutex m_buf;
// To ignore incoming images and imus when
// the state is 'kidnapped'
bool rcvd_tracked_feature = true;
bool rcvd_imu_msg = true;
double deg_to_rad = M_PI / 180.0;

bool virtual_time = false;

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1") {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    } else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

int knd = 0; // how many times inside `if`
int cnd = 0; // how many times kidnapped (mean is low and std dev is low)
int bnd = 0; // how many times in `if` and looks like un-kidnapped

void multi_input_callback(const sensor_msgs::ImageConstPtr &img_msg0,
                          const sensor_msgs::ImageConstPtr &img_msg1) {
    if (!rcvd_tracked_feature) {
        ROS_INFO("[img0_callback] Ignoring Tracked Features");

        // continue publishing /sslam_estimator/keyframe_point.
        knd++;
        if (knd % 8 != 0)
            return;
        // fake_publish( 20 );
        cv::Mat ximage0 = getImageFromMsg(img_msg0);

        cv::Scalar xmean, xstd;
        cv::meanStdDev(ximage0, xmean, xstd);
//        cout << "xmean: " << xmean[0] << "\t" << "xstd: "  << xstd[0] << endl;

        if (xmean[0] < 35. && xstd[0] < 15.) {
            bnd = 0;
            cout << "kidnapped :" << cnd << " xmean: " << xmean[0] << "\t" << "xstd: " << xstd[0] << endl;;
            cnd++;
        } else {
            if (xstd[0] > 20.) {
                cnd = 0;
                cout << "normal    :" << bnd << " xmean: " << xmean[0] << "\t" << "xstd: " << xstd[0] << endl;;
                bnd++;
            }
        }

        if (bnd > 10) {
            cout << "More than THRESH number of consecutive `normals` observed, which means kidnapped mode dismissed\n";
            fake_publish(img_msg0->header, 100);
            return;
        }
        if (cnd > 10) {
//            fake_publish(img_msg0->header, 10);
            return;
        }
        return;
    }
    cnd = 0;
    knd = 0;
    bnd = 0;
    m_buf.lock();
    img0_buf.push(img_msg0);
    img1_buf.push(img_msg1);
    m_buf.unlock();
}

void multi_input_callback_dy(const sensor_msgs::ImageConstPtr &img_msg0,
                             const sensor_msgs::ImageConstPtr &img_msg1,
                             const obstacle_msgs::MapInfoConstPtr &dy_map) {
    if (!rcvd_tracked_feature) {
        ROS_INFO("[img0_callback] Ignoring Tracked Features");

        // continue publishing /sslam_estimator/keyframe_point.
        knd++;
        if (knd % 8 != 0)
            return;
        // fake_publish( 20 );
        cv::Mat ximage0 = getImageFromMsg(img_msg0);

        cv::Scalar xmean, xstd;
        cv::meanStdDev(ximage0, xmean, xstd);
///        cout << "xmean: " << xmean[0] << "\t" << "xstd: "  << xstd[0] << endl;

        if (xmean[0] < 35. && xstd[0] < 15.) {
            bnd = 0;
            cout << "kidnapped :" << cnd << " xmean: " << xmean[0] << "\t" << "xstd: " << xstd[0] << endl;;
            cnd++;
        } else {
            if (xstd[0] > 20.) {
                cnd = 0;
                cout << "normal    :" << bnd << " xmean: " << xmean[0] << "\t" << "xstd: " << xstd[0] << endl;;
                bnd++;
            }
        }

        if (bnd > 10) {
            cout << "More than THRESH number of consecutive `normals` observed, which means kidnapped mode dismissed\n";
            fake_publish(img_msg0->header, 100);
            return;
        }
        if (cnd > 10) {
//            fake_publish(img_msg0->header, 10);
            return;
        }
        return;
    }
    cnd = 0;
    knd = 0;
    bnd = 0;
    m_buf.lock();
    img0_buf.push(img_msg0);
    img1_buf.push(img_msg1);
    dy_buf.push(dy_map);
    m_buf.unlock();
}

/**
* @brief
*   Dynamic object mask information callback.
*/
void dymask_callback(const obstacle_msgs::MapInfoConstPtr &dy_map) {
    m_buf.lock();
    dy_buf.push(dy_map);
    m_buf.unlock();
}

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    if (!rcvd_tracked_feature) {
        ROS_INFO("[img0_callback] Ignoring Tracked Features");

        // continue publishing /sslam_estimator/keyframe_point.
        knd++;
        if (knd % 10 != 0)
            return;
        // fake_publish( 20 );
        cv::Mat ximage0 = getImageFromMsg(img_msg);

        cv::Scalar xmean, xstd;
        cv::meanStdDev(ximage0, xmean, xstd);
        cout << "img0 xmean: " << xmean[0] << "\t" << "xstd: " << xstd[0] << endl;;

        if (xmean[0] < 35. && xstd[0] < 15.)
            cnd++;
        else
            bnd++;

        if (bnd > 10) {
            fake_publish(img_msg->header, 100);
            return;
        }
        if (cnd > 10) {
            fake_publish(img_msg->header, 20);
            return;
        }
        return;
    }
    cnd = 0;
    knd = 0;
    bnd = 0;
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

cv::Mat getMaskFromMsg(const obstacle_msgs::MapInfoConstPtr &dy_map) {
    cv::Mat mask_obs = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    for (const auto &obs : dy_map->obsData) {
        //      0/0---X--->u
        //      |
        //      Y
        //      |
        //      v
        if ((obs.classes != "traffic light") && (obs.classes != "stop sign")
            && (obs.classes != "parking meter") && (obs.classes != "bench")) {
            int dyxmin, dyxmax, dyymin, dyymax;
            dyxmin = std::max(0, static_cast<int>(obs.xmin) - 2);
            dyxmax = std::min(COL, static_cast<int>(obs.xmax) + 2);
            dyymin = std::max(0, static_cast<int>(obs.ymin) - 2);
            dyymax = std::min(ROW, static_cast<int>(obs.ymax) + 2);
            cv::rectangle(mask_obs, cv::Point(dyxmin, dyymin), cv::Point(dyxmax, dyymax), cv::Scalar(0), -1);
        }
    }
    return mask_obs;
}

// extract images with same timestamp from two topics
void sync_process() {
    while (ros::ok()) {
        if (STEREO) {
            cv::Mat image0, image1, mask_dy;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty()) {
//                double time0 = img0_buf.front()->header.stamp.toSec();
//                double time1 = img1_buf.front()->header.stamp.toSec();
//                if(time0 < time1)
//                {
//                    img0_buf.pop();
//                    printf("throw img0\n");
//                }
//                else if(time0 > time1)
//                {
//                    img1_buf.pop();
//                    printf("throw img1\n");
//                }
//                else
//                {

                if(!virtual_time)
                    time = img0_buf.front()->header.stamp.toSec();
                else
                    time  = ros::Time::now().toSec();
//                cout << "image time: " <<  std::fixed << time << endl;
                image0 = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
                image1 = getImageFromMsg(img1_buf.front());
                img1_buf.pop();

                if (CUBICLE) {
                    mask_dy = getMaskFromMsg(dy_buf.front());
                    dy_buf.pop();
                }
//                }
            }
            m_buf.unlock();
            if (!image0.empty()) {
                if (mask_dy.empty()) {
                    estimator.inputImage(time, image0, image1);
                } else
                    estimator.inputImage(time, image0, image1, mask_dy);
            }
        } else {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty()) {
                time = img0_buf.front()->header.stamp.toSec();
//                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if (!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    if (!rcvd_tracked_feature) {
        // ROS_INFO( "Ignoring IMU messages" );
        return;
    }
    double t;
    if(!virtual_time)
        t  = imu_msg->header.stamp.toSec();
    else
        t  = ros::Time::now().toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
}

void ins_callback(const rds_msgs::msg_novatel_inspvaConstPtr &ins_msg) {
    if (!rcvd_tracked_feature) {
        // ROS_INFO( "Ignoring INS messages" );
        return;
    }
    double t;
    if(!virtual_time)
        t  = ins_msg->stamp.toSec();
    else
        t  = ros::Time::now().toSec();
    double dx = ins_msg->east_velocity;
    double dy = ins_msg->north_velocity;
    double dz = ins_msg->up_velocity;
    double rx = ins_msg->roll;
    double ry = ins_msg->pitch;
    double rz = ins_msg->azimuth;
    Vector3d spd(dx, dy, dz);
    Quaterniond ang = AngleAxisd(rz, Eigen::Vector3d::UnitZ())
                    * AngleAxisd(ry, Eigen::Vector3d::UnitY())
                    * AngleAxisd(rx, Eigen::Vector3d::UnitX());
    estimator.inputINS(t, spd, ang, ins_msg->height);
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg) {
    if (!rcvd_tracked_feature) {
        // ROS_INFO( "Ignoring Tracked Features" );
        return;
    }
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++) {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if (feature_msg->channels.size() > 5) {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg) {
    if (restart_msg->data != 0) {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
}

void rcvd_inputs_callback(const std_msgs::BoolConstPtr &rcvd_) {

    if (rcvd_->data != 0 && !rcvd_tracked_feature && !rcvd_imu_msg) {
        ROS_INFO("\n##### rcvd_ set true. So from now on start reading the image and imu messages.");
        rcvd_tracked_feature = true;
        rcvd_imu_msg = true;

        // start the thread
        estimator.clearState();
        estimator.setParameter();
        estimator.processThread_swt = true;
        estimator.startProcessThread();

        return;
    }

    if (rcvd_->data == 0 && rcvd_tracked_feature && rcvd_imu_msg) {
        ROS_INFO("\n###### rcvd_ set false. Will reset the sslam system now");
        rcvd_tracked_feature = false;
        rcvd_imu_msg = false;

        // stop the processing thread.
        estimator.processThread_swt = false;
        estimator.processThread.join();

        // empty the queues and restart the estimator
        m_buf.lock();
        while (!feature_buf.empty())
            feature_buf.pop();
        while (!imu_buf.empty())
            imu_buf.pop();
        while (!img0_buf.empty())
            img0_buf.pop();
        while (!img1_buf.empty())
            img1_buf.pop();
        m_buf.unlock();

        ROS_INFO("all the queues have been emptied");
        estimator.clearState();
        return;
    }

    ROS_INFO("Ignoring rcvd_ message, because it seems invalid.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sslam_estimator_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); // levels::Debug

    std::string config_file;
    n.param("config_path", config_file, ros::package::getPath("sslam") + "/config/bus2/stereo_config.yaml");
    printf("config_file: %s\n", config_file.c_str());
    n.param("virtual_time", virtual_time, false);

    readParameters(config_file);
    estimator.setParameter();
    estimator.processThread_swt = true;
    estimator.startProcessThread();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

//    if(USE_GPS)
//    {
//        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> sharedGPS_info;
//        geometry_msgs::PoseWithCovarianceStamped gps_info;
//        sharedGPS_info = ros::topic::waitForMessage
//                <geometry_msgs::PoseWithCovarianceStamped>(GPS_TOPIC, ros::Duration(20));
//        if(sharedGPS_info != nullptr) {
//            gps_info = *sharedGPS_info;
//
//            gps_0_q = Quaterniond(gps_info.pose.pose.orientation.w, gps_info.pose.pose.orientation.x,
//                                            gps_info.pose.pose.orientation.y, gps_info.pose.pose.orientation.z);
//
//            gps_0_trans = Vector3d(gps_info.pose.pose.position.x,
//                                             gps_info.pose.pose.position.y, gps_info.pose.pose.position.z);
//
//            load_gps_info = true;
//        }
//    }

    registerPub(n);

    //< If you send a true will enable receiving sensor data, if you send false,
    // will start ignoring sensor data
    ros::Subscriber sub_rcvd_flag = n.subscribe("/feature_tracker/rcvd_flag", 2000, rcvd_inputs_callback);
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
//    if(USE_INS)
        ros::Subscriber sub_ins = n.subscribe(INS_TOPIC, 30, ins_callback);
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/slam_restart", 100, restart_callback);

    // Subscribers for the input topics
    message_filters::Subscriber<sensor_msgs::Image> sub_img_l_, sub_img_r_;
    message_filters::Subscriber<obstacle_msgs::MapInfo> cubicle_msg_;

    sub_img_l_.subscribe(n, IMAGE0_TOPIC, 10);
    sub_img_r_.subscribe(n, IMAGE1_TOPIC, 10);

    // Exact time image topic synchronizer
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
            sensor_msgs::Image, obstacle_msgs::MapInfo> ExactPolicy_dy;
    typedef message_filters::Synchronizer<ExactPolicy_dy> ExactSync_dy;
    boost::shared_ptr<ExactSync_dy> exact_sync_dy;

    if (STEREO) {
        if (CUBICLE) {
            cubicle_msg_.subscribe(n, CUBICLE_TOPIC, 3);
            exact_sync_dy.reset(new ExactSync_dy(ExactPolicy_dy(20),
                                                 sub_img_l_,
                                                 sub_img_r_,
                                                 cubicle_msg_));

            exact_sync_dy->registerCallback(boost::bind(
                    &multi_input_callback_dy, _1, _2, _3));
        } else {
            exact_sync_.reset(new ExactSync(ExactPolicy(20),
                                            sub_img_l_,
                                            sub_img_r_));

            exact_sync_->registerCallback(boost::bind(
                    &multi_input_callback, _1, _2));
        }
    } else {
        ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 20, img0_callback);
        ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 20, img1_callback);
        if (CUBICLE)
            ros::Subscriber sub_dynamic = n.subscribe(CUBICLE_TOPIC, 10, dymask_callback);
    }

    std::thread sync_thread{sync_process};
    ros::spin();

    if (estimator.processThread_swt) {
        // join only if the thread is running. Otherwise it will cause
        //  an issue when you attempt to quit in kidnapped mode.
        estimator.processThread_swt = false;
        estimator.processThread.join();
    }
    sync_thread.join();

    return 0;
}
