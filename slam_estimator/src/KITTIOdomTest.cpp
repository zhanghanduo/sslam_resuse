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

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

using namespace slam_estimator;

Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sslam_estimator_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/leftImage", 1000);
    ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/rightImage", 1000);

    if (argc != 3) {
        printf("please intput: rosrun sslam kitti_odom_test [config file] [data folder] \n"
               "for example: rosrun sslam kitti_odom_test "
               "~/catkin_ws/src/sslam_reuse/config/kitti_odom/kitti_config00-02.yaml "
               "/dataset/kitti/odometry/sequences/00/ \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    string sequence = argv[2];
    printf("read sequence: %s\n", argv[2]);
    string dataPath = sequence + "/";

    readParameters(config_file);
    estimator.setParameter();
    registerPub(n);

    // load image list
    FILE *file;
    file = std::fopen((dataPath + "times.txt").c_str(), "r");
    if (file == nullptr) {
        printf("cannot find file: %stimes.txt\n", dataPath.c_str());
        ROS_BREAK();
        return 0;
    }
    double imageTime;
    vector<double> imageTimeList;
    while (fscanf(file, "%lf", &imageTime) != EOF) {
        imageTimeList.push_back(imageTime);
    }
    std::fclose(file);

    string leftImagePath, rightImagePath;
    cv::Mat imLeft, imRight;
    FILE *outFile;
    outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(), "w");
    if (outFile == nullptr)
        printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

    for (size_t i = 0; i < imageTimeList.size(); i++) {
        if (ros::ok()) {
            printf("\nprocess image %d\n", (int) i);
            stringstream ss;
            ss << setfill('0') << setw(6) << i;
            leftImagePath = dataPath + "image_0/" + ss.str() + ".png";
            rightImagePath = dataPath + "image_1/" + ss.str() + ".png";
            //printf("%lu  %f \n", i, imageTimeList[i]);
            //printf("%s\n", leftImagePath.c_str() );
            //printf("%s\n", rightImagePath.c_str() );

            imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE);
            sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
            imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
            pubLeftImage.publish(imLeftMsg);

            imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE);
            sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
            imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
            pubRightImage.publish(imRightMsg);

            estimator.inputImage(imageTimeList[i], imLeft, imRight);

            Eigen::Matrix<double, 4, 4> pose;
            estimator.getPoseInWorldFrame(pose);
            if (outFile != nullptr)
                fprintf(outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n", pose(0, 0), pose(0, 1), pose(0, 2),
                        pose(0, 3),
                        pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
                        pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3));

//			cv::imshow("leftImage", imLeft);
            //cv::imshow("rightImage", imRight);
//			cv::waitKey(0);
        } else
            break;
    }
    if (outFile != nullptr)
        fclose(outFile);
    return 0;
}
