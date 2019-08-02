
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_factor_projectionTwoFrameOneCamFactor.h:

Program Listing for File projectionTwoFrameOneCamFactor.h
=========================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_factor_projectionTwoFrameOneCamFactor.h>` (``/home/ugv/catkin_ws/src/sslam/slam_estimator/src/factor/projectionTwoFrameOneCamFactor.h``)

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
   
   #include <ros/assert.h>
   #include <ceres/ceres.h>
   #include <Eigen/Dense>
   #include "../utility/utility.h"
   #include "../utility/tic_toc.h"
   #include "../estimator/parameters.h"
   
   class ProjectionTwoFrameOneCamFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1> {
   public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
       ProjectionTwoFrameOneCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                                      const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                                      const double _td_i, const double _td_j);
   
       virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
   
       void check(double **parameters);
   
       Eigen::Vector3d pts_i, pts_j;
       Eigen::Vector3d velocity_i, velocity_j;
       double td_i, td_j;
       Eigen::Matrix<double, 2, 3> tangent_base;
       static Eigen::Matrix2d sqrt_info;
       static double sum_t;
   };
