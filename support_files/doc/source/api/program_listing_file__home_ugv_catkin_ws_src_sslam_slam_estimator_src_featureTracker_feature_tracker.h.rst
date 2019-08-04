
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_featureTracker_feature_tracker.h:

Program Listing for File feature_tracker.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_featureTracker_feature_tracker.h>` (``/home/ugv/catkin_ws/src/sslam/slam_estimator/src/featureTracker/feature_tracker.h``)

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
   
   inline double distance(cv::Point2f pt1, cv::Point2f pt2) {
       //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
       double dx = pt1.x - pt2.x;
       double dy = pt1.y - pt2.y;
       return sqrt(dx * dx + dy * dy);
   }
   
   void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
   
   void reduceVector(vector<int> &v, vector<uchar> status);
   
   class FeatureTracker {
   public:
       #ifndef DOXYGEN_SHOULD_SKIP_THIS
           EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   #endif /* DOXYGEN_SHOULD_SKIP_THIS */
   
       FeatureTracker();
   
       map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img,
                                                                           const cv::Mat &_img1 = cv::Mat(),
                                                                           const cv::Mat &_mask = cv::Mat());
   
       void setMask();
   
       void readIntrinsicParameter(const vector<string> &calib_file);
   
       void showUndistortion(const string &name);
   
   
       void rejectWithF();
   
       static vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
   
       vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                       map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
   
       void showTwoImage(const cv::Mat &img1, const cv::Mat &img2,
                         vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
   
       void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                      vector<int> &curLeftIds,
                      vector<cv::Point2f> &curLeftPts,
                      vector<cv::Point2f> &curRightPts,
                      map<int, cv::Point2f> &prevLeftPtsMap);
   
       void setPrediction(map<int, Eigen::Vector3d> &predictPts);
   
       void removeOutliers(set<int> &removePtsIds);
   
       cv::Mat getTrackImage();
   
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
