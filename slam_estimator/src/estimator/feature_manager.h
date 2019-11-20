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

/**
 * @namespace slam_estimator
 */
namespace slam_estimator {

/**
 * @class FeaturePerFrame feature_manager.h
 * @brief Contains all the feature properties including 2D observations, estimated velocity.
 */
    class FeaturePerFrame {
    public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

        /**
         * @brief Construct FeaturePerFrame class with feature properties of left image.
         * @param _point
         * @param td Time interval between this frame and previous frame of the same map point.
         */
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

        /**
         * @brief Add the right image observation to this class.
         * @param _point Feature properties
         */
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

        /**
         * @brief 2D point velocity on left image plane, compared with previous associated feature point.
         */
        Vector2d velocity;

        /**
         * @brief 2D point velocity on right image plane, compared with previous associated feature point.
         */
        Vector2d velocityRight;

        /**
         * @brief true if there is right image observation.
         */
        bool is_stereo;
    };

    /**
     * @class FeaturePerId feature_manager.h
     * @brief Map point indexed by feature_id which contains all feature observations in different frames.
     */
    class FeaturePerId {
    public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

        /**
         * @brief Index of this map point.
         */
        const int feature_id;

        /**
         * @brief The frame index of first observation of this map point.
         */
        int start_frame;

        /**
         * @brief Vector of features in each frame(observation). Its size if the number of observations.
         */
        vector<FeaturePerFrame> feature_per_frame;

        /**
         * @brief Size of @ref feature_per_frame, observation times.
         */
        int used_num;

        /**
         * @brief Estimated depth in meters.
         */
        double estimated_depth;

        /**
         * @brief Flag of depth estimation successful or not.
         * @note 0 haven't solve yet; 1 solve succ; 2 solve fail (estimated depth <0);
         */
        int solve_flag;


        FeaturePerId(int _feature_id, int _start_frame)
                : feature_id(_feature_id), start_frame(_start_frame),
                  used_num(0), estimated_depth(-1.0), solve_flag(0) {
        }

        /**
         * @return The frame index of the last observation of this map point.
         */
        int endFrame();
    };

/**
 * @class FeatureManager feature_manager.h
 * @brief This class stores and calculates observed landmarks into map points.
 * Triangulate feature points and get initial pose estimation after feature tracking @ref FeatureTracker#trackImage().
 */
    class FeatureManager {
    public:
        #ifndef DOXYGEN_SHOULD_SKIP_THIS
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        #endif /* DOXYGEN_SHOULD_SKIP_THIS */

        explicit FeatureManager(Matrix3d _Rs[]);

        /**
         * @brief Update transform between camera and IMU/INS.
         * @param _ric Input extrinsic calibration matrix.
         */
        void setRic(Matrix3d _ric[]);

        /**
         * @brief Clear all the map point memory when resetting.
         */
        void clearState();

        /**
         * @brief Get valid map point number
         * @note Valid means @ref used_num >= \c 4.
         * @return Map point feature size.
         */
        int getFeatureCount();

        /**
         * @brief Add the feature output from @ref trackImage() to @ref feature manager.
         * @param frame_count The index of frame inside sliding window in slam estimator.
         * @param image Not the real "image" but the std::map containing all the feature properties
         * and camera ID as index.
         * @param td Time interval between current and previous message time stamp.
         * @return True if current frame satisfies the keyframe criteria.
         */
        bool
        addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                                double td);

        /**
         * @brief Get the vector of correspondences of frame pairs.
         * @param frame_count_l Frame index 1.
         * @param frame_count_r Frame index 2.
         * @return Correspondence of points based on input frame indices.
         */
        vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

        //void updateDepth(const VectorXd &x);

        /**
         * @brief Set the updated estimated depth into map point database.
         * @param x The estimated inverse depth after optimization.
         */
        void setDepth(const VectorXd &x);

        /**
         * @brief Erase the invalid map points (like estimated depth < 0)
         */
        void removeFailures();

        /**
         * @brief Clear all the records of map point estimated depth with \c -1.
         */
        void clearDepth();

        /**
         * @brief Acquire vector of depth for each valid map point (@ref used_num >= \c 4).
         * @return Vector of inverse estimated depth.
         */
        VectorXd getDepthVector();

        /**
         * @brief Triangulates and calculates the estimated depth for each map point inside @ref feature.
         * @param frameCnt The index of frame inside sliding window in slam estimator.
         * @param Ps Vector of estimated translation vector from all sliding window key frames to world coordinate.
         * @param Rs Vector of estimated rotation matrices from all sliding window key frames to world coordinate.
         * @param tic Extrinsic translation vector from camera to IMU/INS body frame.
         * @param ric Extrinsic rotation matrix from camera to IMU/INS body frame.
         */
        void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);

        /**
         * @brief
         * @param Pose0 Pose of a point at frame 1, from camera plane to world coordinate.
         * @param Pose1 Pose of a point at frame 2, from camera plane to world coordinate.
         * @param point0 2D point coordinate of a point at frame 1, in camera plane.
         * @param point1 2D point coordinate of a point at frame 2, in camera plane.
         * @param[out] point_3d Generated 3D point of this point in world coordinate.
         */
        static void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                              Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);

        /**
         * @brief Refine the vector of estimated Rs, Ps using EPnP.
         * @param frameCnt The index of frame inside sliding window in slam estimator.
         * @param Ps Vector of estimated translation vector from all sliding window key frames to world coordinate.
         * @param Rs Vector of estimated rotation matrices from all sliding window key frames to world coordinate.
         * @param tic Extrinsic translation vector from camera to IMU/INS body frame.
         * @param ric Extrinsic rotation matrix from camera to IMU/INS body frame.
         */
        void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);

        /**
         * @brief Refine pose estimation using cv::SolvePnP.
         * @param R_initial Initial guess of rotation matrix of a frame.
         * @param P_initial Initial guess of translation vector of a frame.
         * @param pts2D Vector of 2D point coordinates in local camera coordinate of a frame.
         * @param pts3D  Corresponding 3D point coordinates in world coordinate.
         * @return True if solve PnP is successful.
         */
        static bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial,
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);

        /**
         * @brief Remove oldest observation of a map point from @ref feature
         * and update initial estimated depth of each point.
         * @note If a map point observation is empty,
         * erase it and update the estimated depth using optimized pose of starting frame.
         * @param marg_R The old initial rotation matrix in a sliding window waiting to be marginalized out.
         * @param marg_P The old initial translation vector in a sliding window waiting to be marginalized out.
         * @param new_R The new initial rotation matrix updated.
         * @param new_P The new initial translation vector updated.
         */
        void removeBackShiftDepth(const Eigen::Matrix3d &marg_R, const Eigen::Vector3d& marg_P,
                                  const Eigen::Matrix3d& new_R, const Eigen::Vector3d new_P);

        /**
         * @brief Remove oldest observation of a map point from @ref feature.
         * @note If the map point observation is empty, erase it directly.
         */
        void removeBack();

        /**
         * @brief Remove the frontest observation of a map point from @ref feature.
         * @param frame_count The index of current frame.
         */
        void removeFront(int frame_count);

        /**
         * @brief Erase outlier map point from @ref feature.
         * @param outlierIndex
         */
        void removeOutlier(set<int> &outlierIndex);

        /**
         * @brief Database of all the map points with feature properties. List of @ref FeaturePerId.
         */
        list<FeaturePerId> feature;

        /**
         * @brief Number of map points that have been observed before.
         */
        int last_track_num;
        double last_average_parallax;

        /**
         * @brief Number of new observed map points.
         */
        int new_feature_num;
        /**
         * @brief Number of map points that have been observed for more than /c times.
         */
        int long_track_num;

    private:
        static double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);

        const Matrix3d *Rs;
        Matrix3d ric[2];
    };
}

#endif