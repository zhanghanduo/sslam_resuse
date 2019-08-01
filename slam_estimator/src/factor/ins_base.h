//
// Created by hd on 22/7/19.
//

#ifndef SRC_INS_BASE_H
#define SRC_INS_BASE_H

#include "../utility/utility.h"
#include "../estimator/parameters.h"

#include <ceres/ceres.h>

using namespace Eigen;

class InterpolationBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InterpolationBase() = delete;

    InterpolationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                     const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg){

    }



};

#endif //SRC_INS_BASE_H
