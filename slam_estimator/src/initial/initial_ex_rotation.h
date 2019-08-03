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
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

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


