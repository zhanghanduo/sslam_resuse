
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_initial_solve_5pts.h:

Program Listing for File solve_5pts.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_initial_solve_5pts.h>` (``/home/ugv/catkin_ws/src/sslam/slam_estimator/src/initial/solve_5pts.h``)

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
   
   using namespace std;
   
   #include <opencv2/opencv.hpp>
   //#include <opencv2/core/eigen.hpp>
   #include <eigen3/Eigen/Dense>
   
   using namespace Eigen;
   
   #include <ros/console.h>
   
   class MotionEstimator {
   public:
       #ifndef DOXYGEN_SHOULD_SKIP_THIS
           EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   #endif /* DOXYGEN_SHOULD_SKIP_THIS */
   
       bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);
   
   private:
       double testTriangulation(const vector<cv::Point2f> &l,
                                const vector<cv::Point2f> &r,
                                cv::Mat_<double> R, cv::Mat_<double> t);
   
       void decomposeE(cv::Mat E,
                       cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                       cv::Mat_<double> &t1, cv::Mat_<double> &t2);
   };
   
   
