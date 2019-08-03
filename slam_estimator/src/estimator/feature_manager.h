/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file feature_manager.h
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief SSLAM-estimator feature database.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>

using namespace std;

#include <eigen3/Eigen/Dense>

using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "../utility/tic_toc.h"

class FeaturePerFrame {
public:
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td) {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5);
        velocity.y() = _point(6);
        cur_td = td;
        is_stereo = false;
    }

    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point) {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5);
        velocityRight.y() = _point(6);
        is_stereo = true;
    }

    /**
     * @brief Time interval between current frame and previous frame.
     */
    double cur_td;
    /**
     * @brief 2D homogeneous coordinate of undistorted point of left image.
     */
    Vector3d point;
    /**
     * @brief 2D homogeneous coordinate of undistorted point of right image.
     */
    Vector3d pointRight;
    /**
     * @brief 2D coordinate of raw point of left image.
     */
    Vector2d uv;
    /**
     * @brief 2D coordinate of raw point of right image.
     */
    Vector2d uvRight;

    Vector2d velocity;

    Vector2d velocityRight;

    bool is_stereo;
};


//***************************************************************************************
//
//! \brief  Map point indexed by feature_id which contains all feature observations in different frames.
//!
//
//***************************************************************************************
class FeaturePerId {
public:
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;
    int used_num;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    FeaturePerId(int _feature_id, int _start_frame)
            : feature_id(_feature_id), start_frame(_start_frame),
              used_num(0), estimated_depth(-1.0), solve_flag(0) {
    }

    int endFrame();
};

class FeatureManager {
public:
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();

    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                                 double td);

    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);

    void removeFailures();

    void clearDepth();

    VectorXd getDepthVector();

    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);

    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                          Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);

    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);

    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial,
                        vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);

    void removeBackShiftDepth(const Eigen::Matrix3d &marg_R, const Eigen::Vector3d marg_P,
                              const Eigen::Matrix3d new_R, const Eigen::Vector3d new_P);

    void removeBack();

    void removeFront(int frame_count);

    void removeOutlier(set<int> &outlierIndex);

    list<FeaturePerId> feature;
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;

private:
    static double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);

    const Matrix3d *Rs;
    Matrix3d ric[2];
};

#endif