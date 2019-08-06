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

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

/**
 * @namespace slam_estimator
 */
namespace slam_estimator {

/**
 * @brief calculates the squared distance between 2 2D points.
 * @param pt1
 * @param pt2
 * @return 2D distance on image plane.
 */
    inline double distance(cv::Point2f pt1, cv::Point2f pt2) {
        //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
        double dx = pt1.x - pt2.x;
        double dy = pt1.y - pt2.y;
        return sqrt(dx * dx + dy * dy);
    }

/**
 * @brief Shrink a vector of cv::Point2f by the valid status vector of same size.
 * @param v Input vector to be reduced.
 * @param status Input vector of valid status, same size of *v*
 */
    void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);

/**
 * @brief Shrink a vector of int by the valid status vector of same size.
 * @param v Input vector to be reduced.
 * @param status Input vector of valid status, same size of *v*
 */
    void reduceVector(vector<int> &v, vector<uchar> status);

/**
 * @class FeatureTracker
 * @brief Visual front end that
 * processes LK optical flow tracking for each feature of each camera.
 */
    class FeatureTracker {
    public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

        FeatureTracker();

        /**
         * @brief Generate feature properties based on input images.
         * @param _cur_time timestamp of the input image.
         * @param _img Input rectified left image (cv::Mat)
         * @param _img1 Input rectified right image (cv::Mat)
         * @param _mask (Optional) Input mask (to filter out dynamic objects)
         * @return Special data structure to store features.
         */
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img,
                                                                            const cv::Mat &_img1 = cv::Mat(),
                                                                            const cv::Mat &_mask = cv::Mat());

        /**
         * @brief 1) Set small masks around last tracked features, with a predefined radius
         * 2) Set "cubicle" mask for dynamic objects.
         */
        void setMask();

        /**
         * @brief Read intrinsic parameters from config file.
         * @param calib_file config file location
         */
        void readIntrinsicParameter(const vector<string> &calib_file);

        void showUndistortion(const string &name);


        /**
         * @brief Fundamental Matrix Estimation with RANSAC to filter out outliers
         */
        void rejectWithF();

        /**
         * @brief Undistort 2D points
         * @param pts vector of 2D image points
         * @param cam camera model of camodocal
         * @return The vector of undistorted points
         */
        static vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);

        /**
         * @brief Calculates the 2D undistorted point velocity
         * @param[in] ids Input vector of IDs
         * @param[in] pts Input vector of 2D point coordinates
         * @param[out] cur_id_pts std::map of ID and current 2D point coordinates
         * @param[in] prev_id_pts std::map of ID and previous 2D point coordinates
         * @return Calculated vector of point velocities of each point
         */
        vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                        map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);

        void showTwoImage(const cv::Mat &img1, const cv::Mat &img2,
                          vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);

        /**
         * @brief Display visualization of image overlaid by optical flows of tracked features.
         * @param imLeft
         * @param imRight
         * @param curLeftIds Vector of current left feature IDs.
         * @param curLeftPts Vector of current left 2D feature point coordinates.
         * @param curRightPts Vector of current right 2D feature point coordinates
         * @param prevLeftPtsMap Vector of previous left 2D point coordinates
         */
        void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                       vector<int> &curLeftIds,
                       vector<cv::Point2f> &curLeftPts,
                       vector<cv::Point2f> &curRightPts,
                       map<int, cv::Point2f> &prevLeftPtsMap);

        /**
         * @brief Predict 2D points based on the 3D predicted points projection.
         * @param[in] predictPts std::map of predicted 3D points associated with IDs
         * @Note There is no return value, but the predict_pts value is the hidden variable to be modified.
         */
        void setPrediction(map<int, Eigen::Vector3d> &predictPts);

        /**
         * @brief Removes the invalid points of vector of "prev_pts", "ids" and "track_cnt" based on flags.
         * @param removePtsIds the set of flags whether the points are outliers or not.
         */
        void removeOutliers(set<int> &removePtsIds);

        /**
         * @brief Inquire the tracked image from external files.
         * @return TrackImage after @ref drawTrack() is implemented.
         */
        cv::Mat getTrackImage();

        /**
         * @brief Judge whether a 2D pixel coordinate cv::Point2f is inside the image
         * @param pt The pixel to be judged.
         * @return flag true if it is inside the image; false otherwise.
         */
        bool inBorder(const cv::Point2f &pt);

        //* row number (height) of the input image.
        int row;
        //* column number (width) of the input image.
        int col;

        //* Image for visualization (with circles and lines to display feature detection and optical flows)
        cv::Mat imTrack;
        //* Image of mask (to filter out dynamic objects and feature associations that are too far away).
        cv::Mat mask;
//    cv::Mat fisheye_mask;
        cv::Mat prev_img, cur_img, dy_mask, dilate_mask_inv;
        vector<cv::Point2f> n_pts;
        vector<cv::Point2f> predict_pts;
        vector<cv::Point2f> predict_pts_debug;
        vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
        vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
        vector<cv::Point2f> pts_velocity, right_pts_velocity;
        vector<int> ids, ids_right;
        vector<int> track_cnt;
        map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
        map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
        map<int, cv::Point2f> prevLeftPtsMap;
        vector<camodocal::CameraPtr> m_camera;
        double cur_time;
        double prev_time;
        bool stereo_cam;
        int n_id;
        bool hasPrediction;
    };
}
