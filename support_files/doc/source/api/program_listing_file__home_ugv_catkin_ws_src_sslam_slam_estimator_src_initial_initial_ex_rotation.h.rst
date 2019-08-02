
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_initial_initial_ex_rotation.h:

Program Listing for File initial_ex_rotation.h
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_initial_initial_ex_rotation.h>` (``/home/ugv/catkin_ws/src/sslam/slam_estimator/src/initial/initial_ex_rotation.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   
   #pragma once
   
   #include <vector>
   #include "../estimator/parameters.h"
   
   using namespace std;
   
   #include <opencv2/opencv.hpp>
   
   #include <eigen3/Eigen/Dense>
   
   using namespace Eigen;
   
   #include <ros/console.h>
   
   /* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
   class InitialEXRotation {
   public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
       InitialEXRotation();
   
       bool
       CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);
   
   private:
       Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);
   
       double testTriangulation(const vector<cv::Point2f> &l,
                                const vector<cv::Point2f> &r,
                                cv::Mat_<double> R, cv::Mat_<double> t);
   
       void decomposeE(cv::Mat E,
                       cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                       cv::Mat_<double> &t1, cv::Mat_<double> &t2);
   
       int frame_count;
   
       vector<Matrix3d> Rc;
       vector<Matrix3d> Rimu;
       vector<Matrix3d> Rc_g;
       Matrix3d ric;  // Inverse of estimated R (imu to world)
   };
   
   
