/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file parameters.h
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief Stores all the external parameters, read from config file.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

//namespace pose_graph {
    extern camodocal::CameraPtr m_camera;
    extern Eigen::Vector3d tic;
    extern Eigen::Matrix3d qic;
//extern ros::Publisher pub_match_img;
    extern int VISUALIZATION_SHIFT_X;
    extern int VISUALIZATION_SHIFT_Y;
    extern std::string BRIEF_PATTERN_FILE;
    extern std::string POSE_GRAPH_SAVE_PATH;
    extern std::string POSE_GRAPH_SAVE_NAME;
    extern int ROW;
    extern int COL;
    extern float ransac_error;
    extern int gps_init;
    extern std::string RESULT_PATH;
    extern int DEBUG_IMAGE;
//}


