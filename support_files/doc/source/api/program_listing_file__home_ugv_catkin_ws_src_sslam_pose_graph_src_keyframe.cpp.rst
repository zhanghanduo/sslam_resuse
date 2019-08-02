
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_keyframe.cpp:

Program Listing for File keyframe.cpp
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_keyframe.cpp>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/keyframe.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*******************************************************
    * Copyright (C) 2019, Robotics Group, Nanyang Technology University
    *
    * \file keyframe.cpp
    * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
    * \date Januarary 2017
    * \brief Each keyframe contains feature descriptors, feature 2D&3D points, and camera realtime pose.
    *
    * Licensed under the GNU General Public License v3.0;
    * you may not use this file except in compliance with the License.
    *
    *******************************************************/
   
   #include "keyframe.h"
   
   template <typename Derived>
   static void reduceVector(vector<Derived> &v, vector<uchar> status)
   {
       int j = 0;
       for (int i = 0; i < int(v.size()); i++)
           if (status[i])
               v[j++] = v[i];
       v.resize(j);
   }
   
   namespace pose_graph {
   
   // create keyframe online
       KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
                          vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv,
                          vector<cv::Point2f> &_point_2d_norm,
                          vector<double> &_point_id, int _sequence) {
           time_stamp = _time_stamp;
           index = _index;
           vio_T_w_i = _vio_T_w_i;
           vio_R_w_i = _vio_R_w_i;
           T_w_i = vio_T_w_i;
           R_w_i = vio_R_w_i;
           origin_vio_T = vio_T_w_i;
           origin_vio_R = vio_R_w_i;
           image = _image.clone();
   //  cv::resize(image, thumbnail, cv::Size(80, 60));
           point_3d = _point_3d;
           point_2d_uv = _point_2d_uv;
           point_2d_norm = _point_2d_norm;
           point_id = _point_id;
           has_loop = false;
           loop_index = -1;
           loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
           sequence = _sequence;
           computeWindowBRIEFPoint();
           computeBRIEFPoint();
           if (!DEBUG_IMAGE)
               image.release();
       }
   
   // load previous keyframe
       KeyFrame::KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i,
                          Matrix3d &_R_w_i,
                          cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1> &_loop_info,
                          vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm,
                          vector<BRIEF::bitset> &_brief_descriptors) {
           time_stamp = _time_stamp;
           index = _index;
           //vio_T_w_i = _vio_T_w_i;
           //vio_R_w_i = _vio_R_w_i;
           vio_T_w_i = _T_w_i;
           vio_R_w_i = _R_w_i;
           T_w_i = _T_w_i;
           R_w_i = _R_w_i;
   //  if (DEBUG_IMAGE)
   //  {
   //      image = _image.clone();
   //      cv::resize(image, thumbnail, cv::Size(80, 60));
   //  }
           has_loop = _loop_index != -1;
           loop_index = _loop_index;
           loop_info = _loop_info;
           sequence = 0;
           keypoints = _keypoints;
           keypoints_norm = _keypoints_norm;
           brief_descriptors = _brief_descriptors;
       }
   
   
       void KeyFrame::computeWindowBRIEFPoint() {
           BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
           for (const auto &i : point_2d_uv) {
               cv::KeyPoint key;
               key.pt = i;
               window_keypoints.push_back(key);
           }
           extractor(image, window_keypoints, window_brief_descriptors);
       }
   
       void KeyFrame::computeBRIEFPoint() {
           BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
           const int fast_th = 20; // corner detector response threshold
   //  if(true)
           cv::FAST(image, keypoints, fast_th, true);
   //  else
   //  {
   //      vector<cv::Point2f> tmp_pts;
   //      cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
   //      for(int i = 0; i < (int)tmp_pts.size(); i++)
   //      {
   //          cv::KeyPoint key;
   //          key.pt = tmp_pts[i];
   //          keypoints.push_back(key);
   //      }
   //  }
           extractor(image, keypoints, brief_descriptors);
           for (auto &keypoint_ : keypoints) {
               Eigen::Vector3d tmp_p;
               m_camera->liftProjective(Eigen::Vector2d(keypoint_.pt.x, keypoint_.pt.y), tmp_p);
               cv::KeyPoint tmp_norm;
               tmp_norm.pt = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
               keypoints_norm.push_back(tmp_norm);
           }
       }
   
       void BriefExtractor::operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys,
                                       vector<BRIEF::bitset> &descriptors) const {
           m_brief.compute(im, keys, descriptors);
       }
   
       bool KeyFrame::searchInArea(const BRIEF::bitset window_descriptor,
                                   const std::vector<BRIEF::bitset> &descriptors_old,
                                   const std::vector<cv::KeyPoint> &keypoints_old,
                                   const std::vector<cv::KeyPoint> &keypoints_old_norm,
                                   cv::Point2f &best_match,
                                   cv::Point2f &best_match_norm) {
           cv::Point2f best_pt;
           int bestDist = 128;
           int bestIndex = -1;
           for (int i = 0; i < (int) descriptors_old.size(); i++) {
   
               int dis = HammingDis(window_descriptor, descriptors_old[i]);
               if (dis < bestDist) {
                   bestDist = dis;
                   bestIndex = i;
               }
           }
           //printf("best dist %d", bestDist);
           if (bestIndex != -1 && bestDist < 80) {
               best_match = keypoints_old[bestIndex].pt;
               best_match_norm = keypoints_old_norm[bestIndex].pt;
               return true;
           } else
               return false;
       }
   
       void KeyFrame::searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                                       std::vector<cv::Point2f> &matched_2d_old_norm,
                                       std::vector<uchar> &status,
                                       const std::vector<BRIEF::bitset> &descriptors_old,
                                       const std::vector<cv::KeyPoint> &keypoints_old,
                                       const std::vector<cv::KeyPoint> &keypoints_old_norm) {
           for (const auto &window_brief_descriptor : window_brief_descriptors) {
               cv::Point2f pt(0.f, 0.f);
               cv::Point2f pt_norm(0.f, 0.f);
               if (searchInArea(window_brief_descriptor, descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
                   status.push_back(1);
               else
                   status.push_back(0);
               matched_2d_old.push_back(pt);
               matched_2d_old_norm.push_back(pt_norm);
           }
       }
   
   
       void KeyFrame::FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                             const std::vector<cv::Point2f> &matched_2d_old_norm,
                                             vector<uchar> &status) {
           int n = (int) matched_2d_cur_norm.size();
           for (int i = 0; i < n; i++)
               status.push_back(0);
           if (n >= 8) {
               vector<cv::Point2f> tmp_cur(n), tmp_old(n);
               for (int i = 0; i < (int) matched_2d_cur_norm.size(); i++) {
                   double FOCAL_LENGTH = 460.0;
                   double tmp_x, tmp_y;
                   tmp_x = FOCAL_LENGTH * matched_2d_cur_norm[i].x + COL / 2.0;
                   tmp_y = FOCAL_LENGTH * matched_2d_cur_norm[i].y + ROW / 2.0;
                   tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);
   
                   tmp_x = FOCAL_LENGTH * matched_2d_old_norm[i].x + COL / 2.0;
                   tmp_y = FOCAL_LENGTH * matched_2d_old_norm[i].y + ROW / 2.0;
                   tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
               }
               cv::findFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 3.0, 0.9, status);
           }
       }
   
       void KeyFrame::PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                                const std::vector<cv::Point3f> &matched_3d,
                                std::vector<uchar> &status,
                                Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old) {
           //for (int i = 0; i < matched_3d.size(); i++)
           //  printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
           //printf("match size %d \n", matched_3d.size());
           cv::Mat r, rvec, t, D, tmp_r;
           cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
           Matrix3d R_inital;
           Vector3d P_inital;
           Matrix3d R_w_c = origin_vio_R * qic;
           Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;
   
           R_inital = R_w_c.inverse();
           P_inital = -(R_inital * T_w_c);
   
           cv::eigen2cv(R_inital, tmp_r);
           cv::Rodrigues(tmp_r, rvec);
           cv::eigen2cv(P_inital, t);
   
           cv::Mat inliers;
           TicToc t_pnp_ransac;
   
           solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, true,
                          100, 0.022, 0.99, inliers);
   
           for (int i = 0; i < (int) matched_2d_old_norm.size(); i++)
               status.push_back(0);
   
           for (int i = 0; i < inliers.rows; i++) {
               int n = inliers.at<int>(i);
               status[n] = 1;
           }
   
           cv::Rodrigues(rvec, r);
           Matrix3d R_pnp, R_w_c_old;
           cv::cv2eigen(r, R_pnp);
           R_w_c_old = R_pnp.transpose();
           Vector3d T_pnp, T_w_c_old;
           cv::cv2eigen(t, T_pnp);
           T_w_c_old = R_w_c_old * (-T_pnp);
   
           PnP_R_old = R_w_c_old * qic.transpose();
           PnP_T_old = T_w_c_old - PnP_R_old * tic;
   
       }
   
   
       bool KeyFrame::findConnection(std::shared_ptr<KeyFrame> &old_kf) {
   //  TicToc tmp_t;
           //printf("find Connection\n");
           vector<cv::Point2f> matched_2d_cur, matched_2d_old;
           vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
           vector<cv::Point3f> matched_3d;
           vector<double> matched_id;
           vector<uchar> status;
   
           matched_3d = point_3d;
           matched_2d_cur = point_2d_uv;
           matched_2d_cur_norm = point_2d_norm;
           matched_id = point_id;
   
   //  TicToc t_match;
           //printf("search by des\n");
           searchByBRIEFDes(matched_2d_old, matched_2d_old_norm, status, old_kf->brief_descriptors, old_kf->keypoints,
                            old_kf->keypoints_norm);
           reduceVector(matched_2d_cur, status);
           reduceVector(matched_2d_old, status);
           reduceVector(matched_2d_cur_norm, status);
           reduceVector(matched_2d_old_norm, status);
           reduceVector(matched_3d, status);
           reduceVector(matched_id, status);
           //printf("search by des finish\n");
   
           status.clear();
           /*
           FundmantalMatrixRANSAC(matched_2d_cur_norm, matched_2d_old_norm, status);
           reduceVector(matched_2d_cur, status);
           reduceVector(matched_2d_old, status);
           reduceVector(matched_2d_cur_norm, status);
           reduceVector(matched_2d_old_norm, status);
           reduceVector(matched_3d, status);
           reduceVector(matched_id, status);
           */
           Eigen::Vector3d PnP_T_old;
           Eigen::Matrix3d PnP_R_old;
           Eigen::Vector3d relative_t;
           Quaterniond relative_q;
           double relative_yaw;
           if ((int) matched_2d_cur.size() > MIN_LOOP_NUM) {
               status.clear();
               PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);
               reduceVector(matched_2d_cur, status);
               reduceVector(matched_2d_old, status);
               reduceVector(matched_2d_cur_norm, status);
               reduceVector(matched_2d_old_norm, status);
               reduceVector(matched_3d, status);
               reduceVector(matched_id, status);
           }
   
           if ((int) matched_2d_cur.size() > MIN_LOOP_NUM) {
               relative_t = PnP_R_old.transpose() * (origin_vio_T - PnP_T_old);
               relative_q = PnP_R_old.transpose() * origin_vio_R;
               relative_yaw = Utility::normalizeAngle(Utility::R2ypr(origin_vio_R).x() - Utility::R2ypr(PnP_R_old).x());
               //printf("PNP relative\n");
               //cout << "pnp relative_t " << relative_t.transpose() << endl;
               //cout << "pnp relative_yaw " << relative_yaw << endl;
               if (abs(relative_yaw) < 30.0 && relative_t.norm() < 20.0) {
   
                   has_loop = true;
                   loop_index = old_kf->index;
                   loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                           relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                           relative_yaw;
                   return true;
               }
           }
           //printf("loop final use num %d %lf--------------- \n", (int)matched_2d_cur.size(), t_match.toc());
           return false;
       }
   
   
       int KeyFrame::HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b) {
           BRIEF::bitset xor_of_bitset = a ^b;
           int dis = xor_of_bitset.count();
           return dis;
       }
   
       void KeyFrame::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i) {
           _T_w_i = vio_T_w_i;
           _R_w_i = vio_R_w_i;
       }
   
       void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i) {
           _T_w_i = T_w_i;
           _R_w_i = R_w_i;
       }
   
       void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i) {
           T_w_i = _T_w_i;
           R_w_i = _R_w_i;
       }
   
       void KeyFrame::updatePose_noz(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i) {
           T_w_i.x() = _T_w_i.x();
           T_w_i.y() = _T_w_i.y();
           R_w_i = _R_w_i;
       }
   
       void KeyFrame::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i) {
           vio_T_w_i = _T_w_i;
           vio_R_w_i = _R_w_i;
           T_w_i = _T_w_i;
           R_w_i = _R_w_i;
       }
   
       void KeyFrame::updateVioPose_noz(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i) {
           vio_R_w_i = _R_w_i;
           vio_T_w_i.x() = _T_w_i.x();
           vio_T_w_i.y() = _T_w_i.y();
           T_w_i.y() = _T_w_i.y();
           T_w_i.x() = _T_w_i.x();
           R_w_i = _R_w_i;
       }
   
       void KeyFrame::getPoints(vector<cv::Point3f> &points_) {
           points_ = point_3d;
       }
   
       void KeyFrame::updatePoints(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i) {
           for (auto &point_ : point_3d) {
               Eigen::Vector3d point_position(point_.x, point_.y, point_.z);
               point_position = _R_w_i * point_position + _T_w_i;
               point_.x = point_position.x();
               point_.y = point_position.y();
               point_.z = point_position.z();
           }
       }
   
       void KeyFrame::updatePoints_noz(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i) {
           for (auto &point_ : point_3d) {
               Eigen::Vector3d point_position(point_.x, point_.y, point_.z);
               point_position = _R_w_i * point_position + _T_w_i;
               point_.x = point_position.x();
               point_.y = point_position.y();
   //        point_.z = point_position.z();
           }
       }
   
       void KeyFrame::reset() {
           local_index = 0;
           has_loop = false;
           sequence = 0;
       }
   
       void KeyFrame::updateEnuPosition(Eigen::Vector3d &_T_w_i) {
           T_enu_i = _T_w_i;
       }
   
   //void KeyFrame::updateEnuPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i)
   //{
   //  T_enu_i = _T_w_i;
   //  R_enu_i = _R_w_i;
   //}
   
       Eigen::Vector3d KeyFrame::getLoopRelativeT() {
           return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
       }
   
       Eigen::Quaterniond KeyFrame::getLoopRelativeQ() {
           return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
       }
   
       double KeyFrame::getLoopRelativeYaw() {
           return loop_info(7);
       }
   
       void KeyFrame::updateLoop(Eigen::Matrix<double, 8, 1> &_loop_info) {
           if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0) {
               //printf("update loop info\n");
               loop_info = _loop_info;
           }
       }
   
       BriefExtractor::BriefExtractor(const std::string &pattern_file) {
           // The DVision::BRIEF extractor computes a random pattern by default when
           // the object is created.
           // We load the pattern that we used to build the vocabulary, to make
           // the descriptors compatible with the predefined vocabulary
   
           // loads the pattern
           cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
           if (!fs.isOpened()) throw string("Could not open file ") + pattern_file;
   
           vector<int> x1, y1, x2, y2;
           fs["x1"] >> x1;
           fs["x2"] >> x2;
           fs["y1"] >> y1;
           fs["y2"] >> y2;
   
           m_brief.importPairs(x1, y1, x2, y2);
       }
   }
