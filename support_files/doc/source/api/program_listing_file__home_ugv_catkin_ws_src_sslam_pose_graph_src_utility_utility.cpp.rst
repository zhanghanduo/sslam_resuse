
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_utility_utility.cpp:

Program Listing for File utility.cpp
====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_utility_utility.cpp>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/utility/utility.cpp``)

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
   
   #include "utility.h"
   
   namespace pose_graph {
   
       Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g) {
           Eigen::Matrix3d R0;
           Eigen::Vector3d ng1 = g.normalized();
           Eigen::Vector3d ng2{0, 0, 1.0};
           R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
           double yaw = Utility::R2ypr(R0).x();
           R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
           // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
           return R0;
       }
   }
