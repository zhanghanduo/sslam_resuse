
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_factor_pose_local_parameterization.h:

Program Listing for File pose_local_parameterization.h
======================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_factor_pose_local_parameterization.h>` (``/home/ugv/catkin_ws/src/sslam/slam_estimator/src/factor/pose_local_parameterization.h``)

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
   
   #include <eigen3/Eigen/Dense>
   #include <ceres/ceres.h>
   #include "../utility/utility.h"
   
   class PoseLocalParameterization : public ceres::LocalParameterization {
       virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
   
       virtual bool ComputeJacobian(const double *x, double *jacobian) const;
   
       virtual int GlobalSize() const { return 7; };
   
       virtual int LocalSize() const { return 6; };
   };
