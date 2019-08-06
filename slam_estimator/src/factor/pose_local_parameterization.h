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
/**
 * @namespace noiseFactor
 */
namespace noiseFactor {
    class PoseLocalParameterization : public ceres::LocalParameterization {
        virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;

        virtual bool ComputeJacobian(const double *x, double *jacobian) const;

        virtual int GlobalSize() const { return 7; };

        virtual int LocalSize() const { return 6; };
    };
}
