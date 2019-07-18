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

#include <stdio.h>
#include <iostream>
#include <algorithm>
//#include <queue>
#include <map>
//#include <thread>
//#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
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
#include <pluginlib/class_list_macros.h>

namespace sslam_estimator {

    class sslam_nodelet : public nodelet::Nodelet {
    public:
        sslam_nodelet() : Nodelet() {};

        ~sslam_nodelet() {}

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

        void multi_input_callback(const sensor_msgs::ImageConstPtr &img_msg0,
                                  const sensor_msgs::ImageConstPtr &img_msg1) {
            cv::Mat image0, image1;
            double time = img_msg0->header.stamp.toSec();
            image0 = getImageFromMsg(img_msg0);
            image1 = getImageFromMsg(img_msg1);

            estimator.inputImage(time, image0, image1);
        }

        void multi_input_callback_dy(const sensor_msgs::ImageConstPtr &img_msg0,
                                     const sensor_msgs::ImageConstPtr &img_msg1,
                                     const obstacle_msgs::MapInfoConstPtr &dy_map) {
            cv::Mat image0, image1, mask_dy;
            double time = img_msg0->header.stamp.toSec();
            image0 = getImageFromMsg(img_msg0);
            image1 = getImageFromMsg(img_msg1);

            if (CUBICLE)
                mask_dy = getMaskFromMsg(dy_map);
            estimator.inputImage(time, image0, image1, mask_dy);
        }

        void img0_callback(const sensor_msgs::ImageConstPtr &img_msg) {
            cv::Mat image0;
            double time = img_msg->header.stamp.toSec();
            image0 = getImageFromMsg(img_msg);
            estimator.inputImage(time, image0);
        }

        cv::Mat getMaskFromMsg(const obstacle_msgs::MapInfoConstPtr &dy_map) {
            cv::Mat mask_obs = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
            for (const auto &obs : dy_map->obsData) {
                //      0/0---X--->u
                //      |
                //      Y
                //      |
                //      v
                //        int xmin_ = std::max(static_cast<int>(obs.xmin) - 10, 0);
                //        int xmax_ = std::min(static_cast<int>(obs.xmax) + 10, COL);
                //        int ymin_ = std::max(static_cast<int>(obs.ymin) - 10, 0);
                //        cv::rectangle(mask_obs, cv::Point(xmin_, ymin_), cv::Point(xmax_, obs.ymax), cv::Scalar(0), -1 );
                cv::rectangle(mask_obs, cv::Point(obs.xmin, obs.ymin), cv::Point(obs.xmax, obs.ymax), cv::Scalar(0),
                              -1);
            }
            return mask_obs;
        }

        void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
            double t = imu_msg->header.stamp.toSec();
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

//        void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
//        {
//            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
//            for (unsigned int i = 0; i < feature_msg->points.size(); i++)
//            {
//                int feature_id = feature_msg->channels[0].values[i];
//                int camera_id = feature_msg->channels[1].values[i];
//                double x = feature_msg->points[i].x;
//                double y = feature_msg->points[i].y;
//                double z = feature_msg->points[i].z;
//                double p_u = feature_msg->channels[2].values[i];
//                double p_v = feature_msg->channels[3].values[i];
//                double velocity_x = feature_msg->channels[4].values[i];
//                double velocity_y = feature_msg->channels[5].values[i];
//                if(feature_msg->channels.size() > 5)
//                {
//                    double gx = feature_msg->channels[6].values[i];
//                    double gy = feature_msg->channels[7].values[i];
//                    double gz = feature_msg->channels[8].values[i];
//                    pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
//                    //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
//                }
//                ROS_ASSERT(z == 1);
//                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//                featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
//            }
//            double t = feature_msg->header.stamp.toSec();
//            estimator.inputFeature(t, featureFrame);
//        }
//
//        void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
//        {
//            if (restart_msg->data != 0)
//            {
//                ROS_WARN("restart the estimator!");
//                estimator.clearState();
//                estimator.setParameter();
//            }
//        }

        virtual void onInit();

    private:
        Estimator estimator;

//        ros::Subscriber sub_imu, sub_feature, sub_restart;

        // Subscribers for the input topics
        message_filters::Subscriber<sensor_msgs::Image> sub_img_l_, sub_img_r_;
        message_filters::Subscriber<obstacle_msgs::MapInfo> cubicle_msg_;

        // Exact time image topic synchronizer
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
        boost::shared_ptr<ExactSync> exact_sync_;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                sensor_msgs::Image, obstacle_msgs::MapInfo> ExactPolicy_dy;
        typedef message_filters::Synchronizer<ExactPolicy_dy> ExactSync_dy;
        boost::shared_ptr<ExactSync_dy> exact_sync_dy;

    };

    void sslam_nodelet::onInit() {

        ros::NodeHandle n_ = getMTPrivateNodeHandle();
        ros::NodeHandle nh_ = getMTNodeHandle();
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); // levels::Debug

        std::string config_file;
        n_.param("config_path", config_file, ros::package::getPath("sslam_estimator") +
                                             "/config/bus2/stereo_config.yaml");
        printf("config_file: %s\n", config_file.c_str());

        readParameters(config_file);
        estimator.setParameter();
        estimator.processThread_swt = true;
        estimator.startProcessThread();

        ROS_WARN("waiting for image and imu...");

        registerPub(n_);

        //< If you send a true will enable receiving sensor data, if you send false,
        // will start ignoring sensor data
        //        sub_imu = nh_.subscribe(IMU_TOPIC, 200, &sslam_nodelet::imu_callback, ros::TransportHints().tcpNoDelay());
        //        sub_feature = nh_.subscribe("/feature_tracker/feature", 200, sslam_nodelet::feature_callback);
        //        sub_restart = nh_.subscribe("/slam_restart", 50, &sslam_nodelet::restart_callback);

        sub_img_l_.subscribe(nh_, IMAGE0_TOPIC, 20);
        sub_img_r_.subscribe(nh_, IMAGE1_TOPIC, 20);

        if (STEREO) {
            if (CUBICLE) {
                cubicle_msg_.subscribe(nh_, CUBICLE_TOPIC, 5);
                exact_sync_dy.reset(new ExactSync_dy(ExactPolicy_dy(80),
                                                     sub_img_l_,
                                                     sub_img_r_,
                                                     cubicle_msg_));

                exact_sync_dy->registerCallback(boost::bind(
                        &sslam_nodelet::multi_input_callback_dy, this, _1, _2, _3));
            } else {
                exact_sync_.reset(new ExactSync(ExactPolicy(30),
                                                sub_img_l_,
                                                sub_img_r_));

                exact_sync_->registerCallback(boost::bind(
                        &sslam_nodelet::multi_input_callback, this, _1, _2));
            }
        }
        //        sync_thread{sync_process};
    }
    // End of class sslam_nodelet
}   // End of namespace sslam_estimator

PLUGINLIB_EXPORT_CLASS(sslam_estimator::sslam_nodelet, nodelet::Nodelet)
