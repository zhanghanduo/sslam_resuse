/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file keyframe.h
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief Each keyframe contains feature descriptors, feature 2D&3D points, and camera realtime pose.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "utility/cerealArchiver.h"
#include "parameters.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"

#define MIN_LOOP_NUM 25

using namespace Eigen;
using namespace std;
using namespace DVision;

namespace pose_graph {
    /**
     * @class BriefExtractor
     * @brief Use brief feature to extract descriptors out of image and keypoints.
     */
    class BriefExtractor {
    public:
        /**
         * @brief Calculates descriptor for keypoints.
         * @param im Input image.
         * @param keys Input corresponding generated vector of keypoints.
         * @param[out] descriptors Calculates corresponding descriptor of keypoints.
         */
        virtual void
        operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;

        /**
         * @brief Constructs brief extractor with external file as pattern.
         */
        BriefExtractor(const std::string &pattern_file);

        /**
         * @brief BRIEF descriptor defined in DVision.
         */
        DVision::BRIEF m_brief;
    };
    /**
     * @class KeyFrame
     * @brief Class to store the necessary information of a keyframe, including index,
     * current pose, key points and corresponding descriptors, etc.
     */
    class KeyFrame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Trivial keyframe class constructor. Not used in the actual code.
         */
        KeyFrame() = default;

        /**
         * @brief New keyframe creation from scratch.
         * @param _time_stamp New keyframe time stamp.
         * @param _index New keyframe index.
         * @param _vio_T_w_i New keyframe translation (vector3d) from current pose to global frame.
         * @param _vio_R_w_i New keyframe rotation (matrix3d) from current pose to global frame.
         * @param _image New keyframe image (for descriptor extraction, released soon).
         * @param _point_3d vector of all the 3D key points belonging to this keyframe.
         * @param _point_2d_uv vector of all the raw 2D key points belonging to this keyframe.
         * @param _point_2d_normal vector of all the homogeneous undistorted 2D key points belonging to this keyframe.
         * @param _point_id vector of the corresponding index of the points.
         * @param _sequence current sequence that the new keyframe should belong to.
         */
        KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
                 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv,
                 vector<cv::Point2f> &_point_2d_normal,
                 vector<double> &_point_id, int _sequence);

        /**
         * @brief Loaded keyframe from prior pose graph map.
         * @param _time_stamp Loaded keyframe time stamp.
         * @param _index Loaded keyframe index.
         * @param _vio_T_w_i Loaded keyframe translation (vector3d) from its pose to global frame.
         * @param _vio_R_w_i Loaded keyframe rotation (matrix3d) from its pose to global frame.
         * @param _T_w_i
         * @param _R_w_i
         * @param _image Loaded keyframe image (for descriptor extraction, released soon).
         * @param _loop_index
         * @param _loop_info
         * @param _keypoints
         * @param _keypoints_norm
         * @param _brief_descriptors
         */
        KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i,
                 Matrix3d &_R_w_i,
                 cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1> &_loop_info,
                 vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm,
                 vector<BRIEF::bitset> &_brief_descriptors);

        /**
         * @brief Find the connection thus acquire relative transform between current frame and the loop frame.
         * @param old_kf
         * @return
         */
        bool findConnection(std::shared_ptr<KeyFrame> &old_kf);

        /**
         * @brief Compute windowed brief descriptors.
         */
        void computeWindowBRIEFPoint();

        void computeBRIEFPoint();

        //void extractBrief();

        /**
         * @brief Hamming distance between two BRIEF descriptors to
         * get the similarity.
         * @param a
         * @param b
         * @return Hamming distance.
         */
        int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);

        /**
         * @brief Search the best match index and point given vector
         * of all previous descriptors.
         * @param window_descriptor Descriptor in a window of the inquiry keyframe.
         * @param descriptors_old The vector of old descriptors.
         * @param keypoints_old The vector of old keypoints.
         * @param keypoints_old_norm The vector of old keypoints (normed).
         * @param[out] best_match Assign the best matched point from keypoints_old.
         * @param[out] best_match_norm Assign the best matched point from keypoints_old_norm.
         * @return true if a best match has been found; false otherwise.
         */
        bool searchInArea(const BRIEF::bitset window_descriptor,
                          const std::vector<BRIEF::bitset> &descriptors_old,
                          const std::vector<cv::KeyPoint> &keypoints_old,
                          const std::vector<cv::KeyPoint> &keypoints_old_norm,
                          cv::Point2f &best_match,
                          cv::Point2f &best_match_norm);

        /**
         * @brief
         * @param matched_2d_old
         * @param matched_2d_old_norm
         * @param status
         * @param descriptors_old
         * @param keypoints_old
         * @param keypoints_old_norm
         */
        void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                              std::vector<cv::Point2f> &matched_2d_old_norm,
                              std::vector<uchar> &status,
                              const std::vector<BRIEF::bitset> &descriptors_old,
                              const std::vector<cv::KeyPoint> &keypoints_old,
                              const std::vector<cv::KeyPoint> &keypoints_old_norm);

        void FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                                    const std::vector<cv::Point2f> &matched_2d_old_norm,
                                    vector<uchar> &status);

        void PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                       const std::vector<cv::Point3f> &matched_3d,
                       std::vector<uchar> &status,
                       Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);

        void getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

        void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

        void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        void updatePose_noz(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        void updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        void updateVioPose_noz(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        void getPoints(vector<cv::Point3f> &p_);

        void updatePoints(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        void updatePoints_noz(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        void reset();

        void updateEnuPosition(Eigen::Vector3d &_T_w_i);

//    void updateEnuPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
        void updateLoop(Eigen::Matrix<double, 8, 1> &_loop_info);

        Eigen::Vector3d getLoopRelativeT();

        double getLoopRelativeYaw();

        Eigen::Quaterniond getLoopRelativeQ();

        double time_stamp;
        int index;
        int local_index;
        Eigen::Vector3d vio_T_w_i;
        Eigen::Matrix3d vio_R_w_i;
        Eigen::Vector3d T_w_i;
        Eigen::Matrix3d R_w_i;
        Eigen::Vector3d origin_vio_T;
        Eigen::Matrix3d origin_vio_R;
        Eigen::Vector3d T_enu_i;
//    Eigen::Matrix3d R_enu_i;
        cv::Mat image;
//	cv::Mat thumbnail;
        vector<cv::Point3f> point_3d;
        vector<cv::Point2f> point_2d_uv;
        vector<cv::Point2f> point_2d_norm;
        vector<double> point_id;
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<cv::KeyPoint> window_keypoints;

        vector<BRIEF::bitset> brief_descriptors;
        /**
         *
         */
        vector<BRIEF::bitset> window_brief_descriptors;

        /**
         * @brief sequence that this keyframe belongs to (normally labeled as 1)
         */
        int sequence;

        bool has_loop;
        int loop_index;
        Eigen::Matrix<double, 8, 1> loop_info;

        template<class Archive>
        void serialize(Archive &ar) {
            ar(CEREAL_NVP(index), CEREAL_NVP(time_stamp),
               CEREAL_NVP(vio_T_w_i), CEREAL_NVP(T_w_i), CEREAL_NVP(vio_R_w_i), CEREAL_NVP(R_w_i),
               CEREAL_NVP(T_enu_i), CEREAL_NVP(loop_index), CEREAL_NVP(loop_info),
               CEREAL_NVP(brief_descriptors), CEREAL_NVP(keypoints), CEREAL_NVP(keypoints_norm), CEREAL_NVP(point_3d));
        }
    };
}
