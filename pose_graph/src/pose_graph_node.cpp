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

#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#define SKIP_FIRST_CNT 2
using namespace std;
using namespace message_filters;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
//std::thread measurement_process;
std::thread keyboard_command_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
//bool load_flag = 0;
//bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;
bool move_mode = false;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string POSE_GRAPH_SAVE_NAME;
std::string RESULT_PATH;
CameraPoseVisualization cameraposevisual(0, 1, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud;

void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("Temporarily only support 5 sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void multi_callback(const sensor_msgs::ImageConstPtr &image_msg_,
        const sensor_msgs::PointCloudConstPtr &point_msg_,
        const nav_msgs::Odometry::ConstPtr &pose_msg_)
{
//    printf("synced!\n");
    if (pose_msg_ != nullptr)
    {
        // skip first few
        if (skip_first_cnt < SKIP_FIRST_CNT) {
            skip_first_cnt++;
            return;
        }

        if (skip_cnt < SKIP_CNT) {
            skip_cnt++;
            return;
        } else
            skip_cnt = 0;

        cv_bridge::CvImageConstPtr ptr;
        if (image_msg_->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = image_msg_->header;
            img.height = image_msg_->height;
            img.width = image_msg_->width;
            img.is_bigendian = image_msg_->is_bigendian;
            img.step = image_msg_->step;
            img.data = image_msg_->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::MONO8);

        cv::Mat image = ptr->image;
        // build keyframe
        Vector3d T = Vector3d(pose_msg_->pose.pose.position.x,
                              pose_msg_->pose.pose.position.y,
                              pose_msg_->pose.pose.position.z);
        Matrix3d R = Quaterniond(pose_msg_->pose.pose.orientation.w,
                                 pose_msg_->pose.pose.orientation.x,
                                 pose_msg_->pose.pose.orientation.y,
                                 pose_msg_->pose.pose.orientation.z).toRotationMatrix();
        if((T - last_t).norm() > SKIP_DIS)
        {
            vector<cv::Point3f> point_3d;
            vector<cv::Point2f> point_2d_uv;
            vector<cv::Point2f> point_2d_normal;
            vector<double> point_id;

            for (unsigned int i = 0; i < point_msg_->points.size(); i++)
            {
                cv::Point3f p_3d;
                p_3d.x = point_msg_->points[i].x;
                p_3d.y = point_msg_->points[i].y;
                p_3d.z = point_msg_->points[i].z;
                point_3d.push_back(p_3d);

                cv::Point2f p_2d_uv, p_2d_normal;
                double p_id;
                p_2d_normal.x = point_msg_->channels[i].values[0];
                p_2d_normal.y = point_msg_->channels[i].values[1];
                p_2d_uv.x = point_msg_->channels[i].values[2];
                p_2d_uv.y = point_msg_->channels[i].values[3];
                p_id = point_msg_->channels[i].values[4];
                point_2d_normal.push_back(p_2d_normal);
                point_2d_uv.push_back(p_2d_uv);
                point_id.push_back(p_id);

                //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
            }

            std::shared_ptr<KeyFrame> keyframe;
            keyframe = std::make_shared<KeyFrame>(pose_msg_->header.stamp.toSec(), frame_index, T, R, image,
                                                  point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
            m_process.lock();
//                start_flag = true;
            posegraph.addKeyFrame(keyframe, true);
            m_process.unlock();
            frame_index ++;
            last_t = T;

//                high_resolution_clock::time_point t2 = high_resolution_clock::now();
//                duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
//                printf("process time: %.1f ms\n", time_span.count() * 1000);
        }
    }

    // for visualization
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg_->header;
    for (auto point : point_msg_->points)
    {
        cv::Point3f p_3d;
        p_3d.x = point.x;
        p_3d.y = point.y;
        p_3d.z = point.z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);

}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    //ROS_INFO("image_callback!");
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 2 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
    // for visualization
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (auto point : point_msg->points)
    {
        cv::Point3f p_3d;
        p_3d.x = point.x;
        p_3d.y = point.y;
        p_3d.z = point.z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);
}

// only for visualization
void margin_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (auto point : point_msg->points) {
        cv::Point3f p_3d;
        p_3d.x = point.x;
        p_3d.y = point.y;
        p_3d.z = point.z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_margin_cloud.publish(point_cloud);
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

void vio_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t( pose_msg->pose.pose.position.x,
                    pose_msg->pose.pose.position.y,
                    pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio * vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    geometry_msgs::PoseWithCovarianceStamped odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = vio_t.x();
    odometry.pose.pose.position.y = vio_t.y();
    odometry.pose.pose.position.z = vio_t.z();
    odometry.pose.pose.orientation.x = vio_q.x();
    odometry.pose.pose.orientation.y = vio_q.y();
    odometry.pose.pose.orientation.z = vio_q.z();
    odometry.pose.pose.orientation.w = vio_q.w();
    odometry.pose.covariance = pose_msg->pose.covariance;
    pub_odometry_rect.publish(odometry);

    if(!move_mode)
        move_mode = true;


    // VIO to camera to output camera TF!
    Vector3d cam_t;
    Quaterniond cam_R;
    cam_R = vio_q * qic;
    cam_t = vio_q * tic + vio_t;

    // Publish body transform w.r.t. world coordinate.
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    transform.setOrigin(tf::Vector3(cam_t(0), cam_t(1), cam_t(2)));
    q.setW(cam_R.w());
    q.setX(cam_R.x());
    q.setY(cam_R.y());
    q.setZ(cam_R.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,
                      pose_msg->header.stamp, "world", "camera"));

    cameraposevisual.reset();
    cameraposevisual.add_pose(cam_t, cam_R);
    cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
}

void process()
{
    while (true)
    {
//        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        sensor_msgs::ImageConstPtr image_msg = nullptr;
        sensor_msgs::PointCloudConstPtr point_msg = nullptr;
        nav_msgs::Odometry::ConstPtr pose_msg = nullptr;

        // find out the messages with same time stamp
//        printf("image 1: %d", image_buf.size());
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec()
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != nullptr)
        {
            // skip first few
            if (skip_first_cnt < SKIP_FIRST_CNT) {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT) {
                skip_cnt++;
                continue;
            } else
                skip_cnt = 0;

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

            cv::Mat image = ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d;
                vector<cv::Point2f> point_2d_uv;
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                std::shared_ptr<KeyFrame> keyframe;
                keyframe = std::make_shared<KeyFrame>(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                m_process.lock();
//                start_flag = true;
                posegraph.addKeyFrame(keyframe, true);
                m_process.unlock();
                frame_index ++;
                last_t = T;

//                high_resolution_clock::time_point t2 = high_resolution_clock::now();
//                duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
//                printf("process time: %.1f ms\n", time_span.count() * 1000);
            }
        }
//        printf("image 2: %d", image_buf.size());

//        if(!move_mode) {
//            // Publish body transform w.r.t. world coordinate.
//            tf::TransformBroadcaster br_0;
//            tf::Transform transform;
//            tf::Quaternion q;
//            // body frame
//            transform.setOrigin(tf::Vector3(0, 0, 0));
//            q.setW(posegraph.gps_0_q.w());
//            q.setX(posegraph.gps_0_q.x());
//            q.setY(posegraph.gps_0_q.y());
//            q.setZ(posegraph.gps_0_q.z());
//            transform.setRotation(q);
//            br_0.sendTransform(tf::StampedTransform(transform,
//                                                  pose_msg->header.stamp, "world", "camera"));
//
//        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    while(ros::ok())
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
//            printf("program shutting down...\n");
//            measurement_process.detach();
//            keyboard_command_process.detach();
//            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(60);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_graph_node");
    ros::NodeHandle n("~");
    ros::NodeHandle nh_pub;
    posegraph.registerPub(nh_pub);
    
    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;

    string config_file;

//    n.param("config_path", config_file, std::string(
//            "/home/hd/catkin_ugv/src/sslam_resuse/slam_estimator/config/bus2/stereo_config.yaml"));
    n.param("config_path", config_file, ros::package::getPath("sslam_estimator") +
                                         "/config/bus2/stereo_config.yaml");
//    printf("config_file: %s\n", argv[1]);
    printf("pose graph (loop fusion) config_file: %s\n", config_file.c_str());

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    cameraposevisual.setScale(3.8);
    cameraposevisual.setLineWidth(0.4);

    std::string IMAGE_TOPIC, GPS_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH, DISPLAY_PREVIOUS_TRAJ;

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    std::string pkg_path = ros::package::getPath("pose_graph");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    printf("cam calib path: %s\n", cam0Path.c_str());
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

    fsSettings["image0_topic"] >> IMAGE_TOPIC;
    fsSettings["gps_topic"] >> GPS_TOPIC;
    fsSettings["pose_graph_save_name"] >> POSE_GRAPH_SAVE_NAME;
    POSE_GRAPH_SAVE_PATH = ros::package::getPath("sslam_estimator") + "/../output";
    fsSettings["save_image"] >> DEBUG_IMAGE;

    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
    DISPLAY_PREVIOUS_TRAJ = fsSettings["display_previous_trajectory"];
    RESULT_PATH = POSE_GRAPH_SAVE_PATH + "/vio_loop.txt";
    std::ofstream fout(RESULT_PATH, std::ios::out);
    fout.close();
    int USE_IMU = fsSettings["imu"];
    int USE_GPS = fsSettings["gps_initial"];
    posegraph.setIMUFlag(USE_IMU);
    posegraph.setTrajFlag(DISPLAY_PREVIOUS_TRAJ);

    cv::Mat cv_T;
    fsSettings["body_T_cam0"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    qic = T.block<3, 3>(0, 0);
    tic = T.block<3, 1>(0, 3);

    fsSettings.release();

    if(USE_GPS)
    {
        printf("Use GPS geoinfo for initial reference.\n Wait for GPS message ...\n");
        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> sharedGPS_info;
        geometry_msgs::PoseWithCovarianceStamped gps_info;
        sharedGPS_info = ros::topic::waitForMessage
                <geometry_msgs::PoseWithCovarianceStamped>(GPS_TOPIC, ros::Duration(20));
        if(sharedGPS_info != nullptr) {
            gps_info = *sharedGPS_info;

            posegraph.gps_0_q = Quaterniond(gps_info.pose.pose.orientation.w, gps_info.pose.pose.orientation.x,
                                            gps_info.pose.pose.orientation.y, gps_info.pose.pose.orientation.z);

            posegraph.gps_0_trans = Vector3d(gps_info.pose.pose.position.x,
                                             gps_info.pose.pose.position.y, gps_info.pose.pose.position.z);

            posegraph.load_gps_info = true;
            printf("Now GPS initial information recorded.");
        } else {
            ROS_WARN("Cannot find GPS topic!");
        }
    }

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("load pose graph finish\n");
//        load_flag = true;
    }
    else
    {
        printf("no previous pose graph\n");
//        load_flag = true;
    }

    std::string vio_sub_topic, keyframe_pose_topic, keypoint_topic, margin_point_topic, extrinsic_topic;


    n.param("vio_odometry", vio_sub_topic, std::string("/sslam_estimator_node/camera_pose"));
    n.param("keyframe_pose", keyframe_pose_topic, std::string("/sslam_estimator_node/keyframe_pose"));
    n.param("keyframe_point", keypoint_topic, std::string("/sslam_estimator_node/keyframe_point"));
    n.param("margin_cloud", margin_point_topic, std::string("/sslam_estimator_node/margin_cloud"));
//    n.param("extrinsic", extrinsic_topic, std::string("/sslam_estimator_node/extrinsic"));

    ros::Subscriber sub_vio = nh_pub.subscribe(vio_sub_topic, 100, vio_callback);
//    ros::Subscriber sub_image = nh_pub.subscribe(IMAGE_TOPIC, 100, image_callback);
//    ros::Subscriber sub_pose = nh_pub.subscribe(keyframe_pose_topic, 100, pose_callback);
//    ros::Subscriber sub_extrinsic = nh_pub.subscribe(extrinsic_topic, 100, extrinsic_callback);
//    ros::Subscriber sub_point = nh_pub.subscribe(keypoint_topic, 100, point_callback);
    ros::Subscriber sub_margin_point = nh_pub.subscribe(margin_point_topic, 100, margin_point_callback);

    Subscriber<sensor_msgs::Image> img_msg_(nh_pub, IMAGE_TOPIC, 30);
    Subscriber<sensor_msgs::PointCloud> pnt_msg_(nh_pub, keypoint_topic, 10);
    Subscriber<nav_msgs::Odometry> pos_msg_(nh_pub, keyframe_pose_topic, 10);

    // Exact time image topic synchronizer
    typedef sync_policies::ApproximateTime
            <sensor_msgs::Image, sensor_msgs::PointCloud, nav_msgs::Odometry> ExactPolicy;
    Synchronizer<ExactPolicy> ExactSync(ExactPolicy(50), img_msg_, pnt_msg_, pos_msg_);
    ExactSync.registerCallback(boost::bind(&multi_callback, _1, _2, _3 ));

//    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 10);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 10);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 10);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 10);
    pub_odometry_rect = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("odometry_rect", 10);

//    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);
    
    ros::spin();

//    measurement_process.detach();
    keyboard_command_process.detach();

    return 0;
}
