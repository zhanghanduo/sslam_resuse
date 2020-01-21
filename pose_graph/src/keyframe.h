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
//#ifdef GPU_FEATURE
//	#include <opencv2/cudaimgproc.hpp>
//	#include <opencv2/cudaarithm.hpp>
//#endif
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "utility/cerealArchiver.h"
#include "parameters.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"

/**
 * @brief The minimum number of matched key points so you can call these two frames are associated.
 */
#define MIN_LOOP_NUM 24

using namespace Eigen;
using namespace std;
using namespace DVision;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
/**
 * @namespace pose_graph
 */
namespace pose_graph {
    /**
     * @class BriefExtractor keyframe.h
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
        explicit BriefExtractor(const std::string &pattern_file);

        /**
         * @brief BRIEF descriptor defined in DVision.
         */
        DVision::BRIEF m_brief;
    };
    /**
     * @class KeyFrame keyframe.h
     * @brief Class to store the necessary information of a keyframe, including index,
     * current pose, key points and corresponding descriptors, etc.
     */
    class KeyFrame {
    public:
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    #endif /* DOXYGEN_SHOULD_SKIP_THIS */

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
        KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image, cv::Mat &_mask_dy,
                 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv,
                 vector<cv::Point2f> &_point_2d_normal,
                 vector<double> &_point_id, int _sequence);

        KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image, cv::Mat &_mask_dy,
                 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv,
                 vector<cv::Point2f> &_point_2d_normal,
                 vector<double> &_point_id, int _sequence, Vector5d& gps_info_);

        /**
         * @brief Loaded keyframe from prior pose graph map.
         * @param _time_stamp Loaded keyframe time stamp.
         * @param _index Loaded keyframe index.
         * @param _vio_T_w_i Loaded keyframe translation (vector3d) from its pose to initial frame.
         * @param _vio_R_w_i Loaded keyframe rotation (matrix3d) from its pose to initial frame.
         * @param _T_w_i Loaded keyframe translation (vector3d) from its pose to global frame.
         * @param _R_w_i Loaded keyframe rotation (matrix3d) from its pose to global frame.
         * @param _image Loaded keyframe image (for descriptor extraction, released soon).
         * @param _loop_index The index of its associated old key frame index.
         * @param _loop_info The relative transform between current and associated old key frame.
         * @param _keypoints Vector of raw key points.
         * @param _keypoints_norm Vector of 2D undistorted key points.
         * @param _brief_descriptors Vector of corresponding brief descriptors of the key points.
         */
        KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i,
                 Matrix3d &_R_w_i, int _loop_index, Eigen::Matrix<double, 8, 1> &_loop_info,
                 vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm,
                 vector<BRIEF::bitset> &_brief_descriptors);

        /**
         * @brief Find the connection thus acquire relative transform between current frame and the loop frame.
         * @param old_kf The old keyframe that was detected by @ref PoseGraph#detectLoop.
         * @return True if loop has been detected.
         */
        bool findConnection(std::shared_ptr<KeyFrame> &old_kf);

        /**
         * @brief Compute windowed brief descriptors according to the corresponding key points.
         * @note Output @ref window_brief_descriptors.
         */
        void computeWindowBRIEFPoint();

        /**
         * @brief Compute fast corner detector and corresponding brief descriptors.
         * @note Output @ref keypoints, @ref keypoints_norm, @ref brief_descriptors.
         */
        void computeBRIEFPoint();

        //void extractBrief();

        /**
         * @brief Hamming distance between two BRIEF descriptors to
         * get the similarity.
         * @return Hamming distance.
         */
        static int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);

        /**
         * @brief Search the best match index and point given vector
         * of all previous descriptors.
         * @param window_descriptor Descriptor in a window of the inquiry keyframe.
         * @param descriptors_old The vector of old descriptors.
         * @param keypoints_old The vector of old keypoints (raw).
         * @param keypoints_old_norm The vector of old keypoints (undistorted).
         * @param[out] best_match Assign the best matched point from keypoints_old.
         * @param[out] best_match_norm Assign the best matched point from keypoints_old_norm.
         * @return true if a best match has been found; false otherwise.
         */
        static bool searchInArea(const BRIEF::bitset& window_descriptor,
                          const std::vector<BRIEF::bitset> &descriptors_old,
                          const std::vector<cv::KeyPoint> &keypoints_old,
                          const std::vector<cv::KeyPoint> &keypoints_old_norm,
                          cv::Point2f &best_match,
                          cv::Point2f &best_match_norm);

        /**
         * @brief Generate the collection of best matches according to current and previous 2D points with descriptors.
         * @param[out] matched_2d_old Collection of points belonging to old frames that best match current frame.
         * @param[out] matched_2d_old_norm Collection of undistorted points belonging to old frames that best match current frame.
         * @param[out] status  Vector of flags representing whether a best match has been found for each point.
         * @param descriptors_old The vector of old descriptors, waiting to be compared.
         * @param keypoints_old The vector of old keypoints (raw).
         * @param keypoints_old_norm The vector of old keypoints (undistorted).
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

        /**
         * @brief Get the inliers and pose of old frame features (to be matched).
         * @param matched_2d_old_norm Vector of 2D image points in image plane.
         * @param matched_3d Vector of 3D landmark points in camera coordinate space.
         * @param[out] status Output vector that represents whether a point is inlier via RANSAC.
         * @param[out] PnP_T_old Translation vector from old landmark frame to initial frame.
         * @param[out] PnP_R_old Rotation matrix from old landmark frame to initial frame.
         */
        void PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                       const std::vector<cv::Point3f> &matched_3d,
                       std::vector<uchar> &status,
                       Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);

        /**
         * @brief Get the VIO pose of this keyframe.
         * @param[out] _T_w_i Translation vector3d from current vio pose to world.
         * @param[out] _R_w_i Rotational matrix3d from current vio pose to world.
         */
        void getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

        /**
         * @brief Get the pose of this keyframe.
         * @param[out] _T_w_i Translation vector3d from current pose to world.
         * @param[out] _R_w_i Rotational matrix3d from current pose to world.
         */
        void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);

        /**
         * @brief Update pose of the keyframe.
         * @param _T_w_i Input translation vector3d from current pose to world.
         * @param _R_w_i Input rotational matrix3d from current pose to world.
         */
        void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        /**
         * @brief Update pose of the keyframe ignoring z axis.
         * @param _T_w_i Input translation vector3d from current pose to world.
         * @param _R_w_i Input rotational matrix3d from current pose to world.
         */
        void updatePose_noz(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        /**
         * @brief Update pose together with VIO pose of the keyframe.
         * @param _T_w_i Input translation vector3d from current pose to world.
         * @param _R_w_i Input rotational matrix3d from current pose to world.
         */
        void updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        /**
         * @brief Update pose together with VIO pose of the keyframe ignoring rotation.
         * @param _T_w_i Input translation vector3d from current pose to world.
         */
        void updateVioPose_norot(const Eigen::Vector3d &_T_w_i);

        /**
         * @brief Get the vector of 3D landmark points of this keyframe.
         * @param[out] p_ vector of Point3f points.
         */
        void getPoints(vector<cv::Point3f> &p_);

        /**
         * @brief Update the 3D pose of all landmarks via input transform.
         * @param _T_w_i Input translational vector.
         * @param _R_w_i Input rotational matrix.
         */
        void updatePoints(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        /**
         * @brief Update the 3D pose of all landmarks via input transform, ignoring rotation.
         * @param _T_w_i Input translational vector.
         */
        void updatePoints_norot(const Eigen::Vector3d &_T_w_i);

        /**
         * @brief Clear the loop and sequence information, only used when loading old keyframes.
         */
        void reset();

        /**
         * Update camera position under ENU location, only used when loading old keyframes.
         * @param _T_w_i
         */
        void updateEnuPosition(Eigen::Vector3d &_T_w_i);

//    void updateEnuPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

        /**
         * @brief Replace loop info (relative transform) using input param.
         * @param _loop_info
         */
        void updateLoop(Eigen::Matrix<double, 8, 1> &_loop_info);

        /**
         * @brief Acquire the relative translation transform between current keyframe and its loop keyframe.
         * @return Relative translation vector3d.
         */
        Eigen::Vector3d getLoopRelativeT();

        /**
         * @brief Acquire the relative yaw angle in degree unit between current keyframe and its loop keyframe.
         * @return Yaw angle.
         */
        double getLoopRelativeYaw();

        /**
         * @brief Acquire the relative rotation quaternion between current keyframe and its loop keyframe.
         * @return Relative quaterniond.
         */
        Eigen::Quaterniond getLoopRelativeQ();

        /**
         * @brief The keyframe creation time stamp.
         */
        double time_stamp;
        /**
         * @brief The keyframe unique index. For prior map case, the starting index is bigger than 1.
         */
        int index;
        /**
         * @brief Local index is only used in each time optimization.
         * They are cleared and recalculated each time optimization is started.
         */
        int local_index;

        /**
         * @brief The current translation vector from current IMU/INS frame to initial frame.
         */
        Eigen::Vector3d vio_T_w_i;
        /**
         * @brief The current rotation matrix from current IMU/INS frame to initial frame.
         */
        Eigen::Matrix3d vio_R_w_i;

        /**
         * @brief The current translation vector from current IMU/INS frame to world frame.
         * @note Same as @ref vio_T_w_i if no prior map is loaded. The final output is T_w_i and R_w_i.
         */
        Eigen::Vector3d T_w_i;
        /**
         * @brief The current rotation matrix from current IMU/INS frame to world frame.
         * @note Same as @ref vio_T_w_i if no prior map is loaded. The final output is T_w_i and R_w_i.
         */
        Eigen::Matrix3d R_w_i;

        /**
         * @brief The original VIO translation vector.
         * @note If no loop detected, it is equal to @ref vio_T_w_i.
         */
        Eigen::Vector3d origin_vio_T;

        /**
         * @brief The original VIO rotation matrix.
         * @note If no loop detected, it is equal to @ref vio_R_w_i.
         */
        Eigen::Matrix3d origin_vio_R;

        /**
         * @brief The translation vector from current keyframe to ENU frame.
         * @note It is only used for initial alignment. ENU frame info comes
         * from GPS initial message.
         */
        Eigen::Vector3d T_enu_i;
//    Eigen::Matrix3d R_enu_i;
        /**
         * @brief Image of current frame for brief descriptor calculation.
         * Released soon after that for memory efficiency.
         */
        cv::Mat image;

        cv::Mat mask_dy;
//	cv::Mat thumbnail;

        /**
         * @brief All the 3D landmark points received from @ref FeaturePerId#feature_per_frame output.
         * @note It is calculated by feature_per
         */
        vector<cv::Point3f> point_3d;

        /**
         * @brief 2D raw points under image plane from @ref FeaturePerId#feature_per_frame#uv output
         */
        vector<cv::Point2f> point_2d_uv;

        /**
         * @brief 2D undistorted points under camera coordinate frame from @ref fFeaturePerId#eature_per_frame#point output
         */
        vector<cv::Point2f> point_2d_norm;

        /**
         * @brief feature ID from @ref FeaturePerId#feature_per_frame#feature_id output
         */
        vector<double> point_id;

        /**
         * @brief Image key points in image plane generated by @ref computeBRIEFPoint().
         */
        vector<cv::KeyPoint> keypoints;

        /**
         * @brief Image key points in camera coordinate frame generated by @ref computeBRIEFPoint().
         */
        vector<cv::KeyPoint> keypoints_norm;

        /**
         * @brief Windowed key points received from @ref point_2d_uv and generated by @ref computeWindowBRIEFPoint().
         */
        vector<cv::KeyPoint> window_keypoints;

        /**
         * @brief Key point descriptors generated by @ref computeBRIEFPoint().
         */
        vector<BRIEF::bitset> brief_descriptors;

        /**
         * @brief Windowed key point descriptors received from @ref point_2d_uv and generated by @ref computeWindowBRIEFPoint().
         * It is only used for temporary comparison with loop keyframe's @ref brief_descriptors.
         */
        vector<BRIEF::bitset> window_brief_descriptors;

        /**
         * @brief sequence that this keyframe belongs to (normally labeled as 1)
         */
        int sequence;

        /**
         * @brief True means this keyframe has an old keyframe loop detection.
         */
        bool has_loop;

        bool has_gps;
        /**
         * @brief True means this keyframe is a standalone loop detection, without previous concecutive detections.
         */
        bool standalone;

        Vector5d gps_info;

        /**
         * @brief If @ref has_loop is true, loop_index means the index of that associated old keyframe.
         */
        int loop_index;
        /**
         * @brief The vector containing the loop detection information.
         * @details 8 elements, respectively containing 3d relative translation from this keyframe to old keyframe,
         * 4d relative rotation quaternion, and 1d yaw angle in degree.
         */
        Eigen::Matrix<double, 8, 1> loop_info;

        /**
         * @brief Record the variables to be serialized / de-serialized.
         * @tparam Archive Format of serializing (binary or XML or JSON).
         * @param ar The target to be serialized / de-serialized.
         */
        template<class Archive>
        void serialize(Archive &ar) {
            ar(CEREAL_NVP(index), CEREAL_NVP(time_stamp),
               CEREAL_NVP(vio_T_w_i), CEREAL_NVP(T_w_i), CEREAL_NVP(vio_R_w_i), CEREAL_NVP(R_w_i),
               CEREAL_NVP(T_enu_i), CEREAL_NVP(loop_index), CEREAL_NVP(loop_info),
               CEREAL_NVP(brief_descriptors), CEREAL_NVP(keypoints), CEREAL_NVP(keypoints_norm), CEREAL_NVP(point_3d));
        }
    };
}
