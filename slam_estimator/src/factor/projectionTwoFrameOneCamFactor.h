/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file projectionTwoFrameOneCamFactor.h
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief Config parameters read from external config files.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"
/**
 * @namespace noiseFactor
 */
namespace noiseFactor {
    /**
     * @class ProjectionTwoFrameOneCamFactor
     * @brief Derived from ceres cost function to describe the error between two observations
     * from two frames considering both 2D homogeneous point coordinates and 2D velocities.
     */
    class ProjectionTwoFrameOneCamFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1> {
    public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

        ProjectionTwoFrameOneCamFactor(Eigen::Vector3d _pts_i, Eigen::Vector3d _pts_j,
                                       const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                                       const double _td_i, const double _td_j);

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

        void check(double **parameters);

        Eigen::Vector3d pts_i, pts_j;
        Eigen::Vector3d velocity_i, velocity_j;
        double td_i, td_j;
        Eigen::Matrix<double, 2, 3> tangent_base;
        static Eigen::Matrix2d sqrt_info;
//        static double sum_t;
    };
}