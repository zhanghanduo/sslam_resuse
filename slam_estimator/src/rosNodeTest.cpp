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
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// Obstacle ros msgs
#include <obstacle_msgs/MapInfo.h>
#include <obstacle_msgs/obs.h>
#include <obstacle_msgs/point3.h>

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<obstacle_msgs::MapInfoConstPtr> dy_buf;
queue<geometry_msgs::PoseWithCovarianceStampedConstPtr> gps_buf;
std::mutex m_buf;
bool gps_initialized = false;
Eigen::Vector3d p_offset;
Eigen::Quaterniond q_offset;

void multi_input_callback(const sensor_msgs::ImageConstPtr &img_msg0,
                          const sensor_msgs::ImageConstPtr &img_msg1)
{
//    if(image_0_cnt % 2 != 0) {
        m_buf.lock();
        img0_buf.push(img_msg0);
        img1_buf.push(img_msg1);
        m_buf.unlock();
//    }
//    image_0_cnt ++;
}

void multi_input_callback_dy(const sensor_msgs::ImageConstPtr &img_msg0,
                             const sensor_msgs::ImageConstPtr &img_msg1,
                             const obstacle_msgs::MapInfoConstPtr& dy_map)
{
//    if(image_0_cnt % 3 != 0) {
    m_buf.lock();
    img0_buf.push(img_msg0);
    img1_buf.push(img_msg1);
    dy_buf.push(dy_map);
    m_buf.unlock();
//    }
//    image_0_cnt ++;
}

void multi_input_callback_gps(const sensor_msgs::ImageConstPtr &img_msg0,
                             const sensor_msgs::ImageConstPtr &img_msg1,
                             const geometry_msgs::PoseWithCovarianceStampedConstPtr& gps_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg0);
    img1_buf.push(img_msg1);
    gps_buf.push(gps_msg);
    m_buf.unlock();
}

  /**
  * @brief
  *   Dynamic object mask information callback.
  */
void dymask_callback(const obstacle_msgs::MapInfoConstPtr& dy_map)
{
    m_buf.lock();
    dy_buf.push(dy_map);
    m_buf.unlock();
}

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
//    if(image_0_cnt % 3 != 0) {
        m_buf.lock();
        img0_buf.push(img_msg);
        m_buf.unlock();
//    }
//    image_0_cnt ++;
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
//    if(image_1_cnt % 3 != 0) {
        m_buf.lock();
        img1_buf.push(img_msg);
        m_buf.unlock();
//    }
//    image_1_cnt ++;
}

cv::Mat getMaskFromMsg(const obstacle_msgs::MapInfoConstPtr &dy_map)
{
    cv::Mat mask_obs = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    for(const auto &obs : dy_map->obsData)
    {
        //      0/0---X--->u
        //      |
        //      Y
        //      |
        //      v
//        int xmin_ = std::max(static_cast<int>(obs.xmin) - 10, 0);
//        int xmax_ = std::min(static_cast<int>(obs.xmax) + 10, COL);
//        int ymin_ = std::max(static_cast<int>(obs.ymin) - 10, 0);
//        cv::rectangle(mask_obs, cv::Point(xmin_, ymin_), cv::Point(xmax_, obs.ymax), cv::Scalar(0), -1 );
        cv::rectangle(mask_obs, cv::Point(obs.xmin, obs.ymin), cv::Point(obs.xmax, obs.ymax), cv::Scalar(0), -1 );
    }
    return mask_obs;
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

std::vector<double> getGlobalPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &gps_msg)
{
    double x_ = gps_msg->pose.pose.position.x;
    double y_ = gps_msg->pose.pose.position.y;
    double z_ = gps_msg->pose.pose.position.z;
    Eigen::Vector3d p_(x_, y_, z_);
    Eigen::Quaterniond q_(gps_msg->pose.pose.orientation.w, gps_msg->pose.pose.orientation.x,
                          gps_msg->pose.pose.orientation.y, gps_msg->pose.pose.orientation.z);

    if(!gps_initialized) {
        p_offset = p_;
        q_offset = q_.inverse();
        p_ = Eigen::Vector3d::Zero();
        q_ = Eigen::Quaterniond::Identity();
        gps_initialized = true;
    } else {
        p_.x() = x_ - p_offset.x();
        p_.y() = y_ - p_offset.y();
        p_.z() = z_ - p_offset.z();
        q_ = q_offset * q_;
        q_.normalize();
    }

    std::vector<double> tmp{p_.x(), p_.y(), p_.z(), q_.w(), q_.x(), q_.y(), q_.z()};
    return tmp;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(ros::ok())
    {
        if(STEREO)
        {
            cv::Mat image0, image1, mask_dy;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
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
                    time = img0_buf.front()->header.stamp.toSec();
//                    cout << "time: " <<  std::fixed << time << endl;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();

                    if(CUBICLE)
                    {
                        mask_dy = getMaskFromMsg(dy_buf.front());
                        dy_buf.pop();
                    }
                    if(USE_GPS)
                    {
                        std::vector<double> gps_msg = getGlobalPose(gps_buf.front());
                        estimator.getGPS(time, gps_msg);
                        gps_buf.pop();
                    }
//                }
            }
            m_buf.unlock();
            if(!image0.empty()) {
                if(mask_dy.empty())
                    estimator.inputImage(time, image0, image1);
                else
                    estimator.inputImage(time, image0, image1, mask_dy);
            }
        } else {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
//                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
                if(USE_GPS)
                {
                    std::vector<double> gps_msg = getGlobalPose(gps_buf.front());
                    estimator.getGPS(time, gps_msg);
                    gps_buf.pop();
                }
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
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

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sslam_estimator_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::string config_file;
    n.param("config_path", config_file, std::string("/home/hd/catkin_ugv/src/sslam_resuse/slam_estimator/config/bus2/stereo_config.yaml"));
    printf("config_file: %s\n", config_file.c_str());

    readParameters(config_file);
    estimator.setParameter();

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

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/slam_restart", 100, restart_callback);
    
    // Subscribers for the input topics
    message_filters::Subscriber<sensor_msgs::Image> sub_img_l_, sub_img_r_;
    message_filters::Subscriber<obstacle_msgs::MapInfo> cubicle_msg_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> gps_msg_;

    sub_img_l_.subscribe(n, IMAGE0_TOPIC, 3);
    sub_img_r_.subscribe(n, IMAGE1_TOPIC, 3);

    // Exact time image topic synchronizer
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                             sensor_msgs::Image, obstacle_msgs::MapInfo> ExactPolicy_dy;
    typedef message_filters::Synchronizer<ExactPolicy_dy> ExactSync_dy;
    boost::shared_ptr<ExactSync_dy> exact_sync_dy;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
            sensor_msgs::Image, geometry_msgs::PoseWithCovarianceStamped> ApproximatePolicy_g;
    typedef message_filters::Synchronizer<ApproximatePolicy_g> ApproximateSync_g;
    boost::shared_ptr<ApproximateSync_g> approximate_sync_g;

    if(STEREO) {
        if(CUBICLE) {
            cubicle_msg_.subscribe(n, CUBICLE_TOPIC, 20);
            exact_sync_dy.reset( new ExactSync_dy( ExactPolicy_dy(80),
                                              sub_img_l_,
                                              sub_img_r_,
                                              cubicle_msg_) );

            exact_sync_dy->registerCallback( boost::bind(
                    &multi_input_callback_dy, _1, _2, _3 ) );
        } else if (USE_GPS) {   // Currently does not support both cubicle filtering and gps integration
            gps_msg_.subscribe(n, GPS_TOPIC, 100);
            approximate_sync_g.reset( new ApproximateSync_g( ApproximatePolicy_g(80),
                                                sub_img_l_,
                                                sub_img_r_,
                                                gps_msg_) );
            approximate_sync_g->registerCallback( boost::bind(
                    &multi_input_callback_gps, _1, _2, _3 ) );

        } else {
            exact_sync_.reset( new ExactSync( ExactPolicy(10),
                                              sub_img_l_,
                                              sub_img_r_ ) );

            exact_sync_->registerCallback( boost::bind(
                    &multi_input_callback, _1, _2 ) );
        }
    } else {
            ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
            ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
            if(CUBICLE)
                ros::Subscriber sub_dynamic = n.subscribe(CUBICLE_TOPIC, 10, dymask_callback);
    }

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
