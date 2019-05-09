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

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car, pub_point_cloud, pub_margin_cloud, pub_keyframe_odometry;
nav_msgs::Path *global_path;

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Quaterniond rot_(0, 0, 0.7071068, 0.7071068);
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot_;
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 1.2;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    // for visualization
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    point_cloud.channels = point_msg->channels;
    for (auto point : point_msg->points)
    {
        Eigen::Vector3d p_3d;
        p_3d.x() = point.x;
        p_3d.y() = point.y;
        p_3d.z() = point.z;
        Eigen::Vector3d tmp = globalEstimator.WGPS_T_WVIO.block<3, 3>(0, 0) * p_3d + globalEstimator.WGPS_T_WVIO.block<3, 1>(0, 3);
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);
}

void keypose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    nav_msgs::Odometry pose_odom_;
    pose_odom_.header = pose_msg->header;

    Eigen::Vector3d global_t, local_t;
    Eigen:: Quaterniond global_q, local_q;
    local_t.x() = pose_msg->pose.pose.position.x;
    local_t.y() = pose_msg->pose.pose.position.y;
    local_t.z() = pose_msg->pose.pose.position.z;
    local_q.x() = pose_msg->pose.pose.orientation.x;
    local_q.y() = pose_msg->pose.pose.orientation.y;
    local_q.z() = pose_msg->pose.pose.orientation.z;
    local_q.w() = pose_msg->pose.pose.orientation.w;

    global_q = globalEstimator.WGPS_T_WVIO.block<3, 3>(0, 0) * local_q;
    global_t = globalEstimator.WGPS_T_WVIO.block<3, 3>(0, 0) * local_t + globalEstimator.WGPS_T_WVIO.block<3, 1>(0, 3);
    pose_odom_.pose.pose.position.x = global_t.x();
    pose_odom_.pose.pose.position.y = global_t.y();
    pose_odom_.pose.pose.position.z = global_t.z();
    pose_odom_.pose.pose.orientation.x = global_q.x();
    pose_odom_.pose.pose.orientation.y = global_q.y();
    pose_odom_.pose.pose.orientation.z = global_q.z();
    pose_odom_.pose.pose.orientation.w = global_q.w();
    pub_keyframe_odometry.publish(pose_odom_);
}

// only for visualization
void margin_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (auto point : point_msg->points) {
        Eigen::Vector3d p_3d;
        p_3d.x() = point.x;
        p_3d.y() = point.y;
        p_3d.z() = point.z;
        Eigen::Vector3d tmp = globalEstimator.WGPS_T_WVIO.block<3, 3>(0, 0) * p_3d + globalEstimator.WGPS_T_WVIO.block<3, 1>(0, 3);
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_margin_cloud.publish(point_cloud);
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    //printf("GPS_callback! \n");
    double t = GPS_msg->header.stamp.toSec();
    //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = GPS_msg->altitude;
    //int numSats = GPS_msg->status.service;
    double pos_accuracy = GPS_msg->position_covariance[0];
    //printf("receive covariance %lf \n", pos_accuracy);
    globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
}

void GPS_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & gps_pose)
{
//    printf("GPS_callback! \n");
    double t = gps_pose->header.stamp.toSec();

    double x_ = gps_pose->pose.pose.position.x;
    double y_ = gps_pose->pose.pose.position.y;
    double z_ = gps_pose->pose.pose.position.z;
    double pos_accuracy = gps_pose->pose.covariance[0];
    globalEstimator.inputGPS_xyz(t, x_, y_, z_, pos_accuracy);
}

void vio_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg,
                  const geometry_msgs::PoseWithCovarianceStampedConstPtr & gps_pose)
{
//    printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();

    double x_ = gps_pose->pose.pose.position.x;
    double y_ = gps_pose->pose.pose.position.y;
    double z_ = gps_pose->pose.pose.position.z;
//    double pos_accuracy = gps_pose->pose.covariance[0];
    double pos_accuracy = 0.0022;
    globalEstimator.inputGPS_xyz(t, x_, y_, z_, pos_accuracy);

    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x,
                          pose_msg->pose.pose.position.y,
                          pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);

//    if(!globalEstimator.initGPS)
//        globalEstimator.offset = vio_t;

//    printf("pose x: %f y: %f z: %f\n", vio_t.x(), vio_t.y(), vio_t.z());

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    geometry_msgs::PoseWithCovarianceStamped odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
//    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");
    ros::NodeHandle n_pub;

    global_path = &globalEstimator.global_path;

    std::string odo_topic, gps_topic, keyframe_pose_topic, keypoint_topic, margin_point_topic;
    n.param("odometry_topic", odo_topic, std::string("/sslam_estimator_node/camera_pose"));
    n.param("gps_topic", gps_topic, std::string("/gps/pose"));
    n.param("keyframe_pose", keyframe_pose_topic, std::string("/sslam_estimator_node/keyframe_pose"));
    n.param("keyframe_point", keypoint_topic, std::string("/sslam_estimator_node/keyframe_point"));
    n.param("margin_cloud", margin_point_topic, std::string("/sslam_estimator_node/margin_cloud"));
//    ros::Subscriber sub_GPS = n.subscribe("/gps_pose", 100, GPS_pose_callback);
//    ros::Subscriber sub_vio = n.subscribe("/sslam_estimator_node/odometry", 100, vio_callback);
    ros::Subscriber sub_margin_point = n.subscribe(margin_point_topic, 2000, margin_point_callback);
    ros::Subscriber sub_point = n.subscribe(keypoint_topic, 2000, point_callback);
    ros::Subscriber sub_keyframe = n.subscribe(keyframe_pose_topic, 2000, keypose_callback);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_vio_(n_pub, odo_topic, 50);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_GPS_(n_pub, gps_topic, 20);

//     Approximate time gps and vio topic synchronizer
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped,
            geometry_msgs::PoseWithCovarianceStamped> ApproximatePolicy;

    message_filters::Synchronizer<ApproximatePolicy> ApproximateSync(ApproximatePolicy(50),
            sub_vio_, sub_GPS_);

    ApproximateSync.registerCallback( boost::bind( &vio_callback, _1, _2 ) );

    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("global_point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("global_margin_cloud", 1000);
    pub_keyframe_odometry = n.advertise<nav_msgs::Odometry>("global_keyframe_pose", 100);
    ros::spin();
    return 0;
}
