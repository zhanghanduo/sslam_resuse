/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file pose_graph.h
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief Pose graph saving &loading, optimization and merging.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include <thread>
#include <mutex>
#include <memory>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <queue>
#include <assert.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <ros/ros.h>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"
#include "utility/cerealArchiver.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"

// Draw local connection
#define SHOW_S_EDGE true
// Draw loop closure connection
#define SHOW_L_EDGE false
#define SAVE_LOOP_PATH true

using namespace DVision;
using namespace DBoW2;

namespace pose_graph {
    /**
     * @class PoseGraph
     * @brief Handles reading/writing, loading/saving and optimization of key frames.
     */
    class PoseGraph {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// \brief Pose graph class constructor haha.
        /// \details Handles reading/writing, loading/saving and optimization of key frames.
        /// \param none.
        PoseGraph();

        /// \brief Pose graph class destructor.
        /// \param none.
        ~PoseGraph();

        /// \brief Get ready for the output path, pointcloud, odometry, visual marker in ROS format.
        /// \param[in] ROS node handler passed from external ROS node main function.
        void registerPub(ros::NodeHandle &n);

        /// \brief Add observed new keyframe into the keyframe database.
        /// \param[in] keyframe pointer. After KeyFrame() constructor, we get a shared_ptr of keyframe.
        /// \param[in] Flag of whether detecting a loop.
        void addKeyFrame(std::shared_ptr<KeyFrame> &cur_kf, bool flag_detect_loop);

        /// \brief Add loaded keyframe from previously stored prior map into the current keyframe database.
        /// \param[in] keyframe pointer. Loaded "old" keyframe.
        /// \param[in] Flag of whether detecting a loop. (Normally set to "false")
        void loadKeyFrame(std::shared_ptr<KeyFrame> &cur_kf, bool flag_detect_loop);

        /// \brief Load vocabulary of DBOW2 training samples for future loop recognition.
        /// \param[in] Vocabulary file location.
        void loadVocabulary(std::string voc_path);

        /// \brief Proceed optimization based on the type of VO source. VIO directly uses 4D and VO uses 6D.
        /// \param[in] Flag of whether the VO source has inertial sensor.
        void setIMUFlag(bool _use_imu);

        /// \brief Decide whether displaying the prior map trajectory for visualization.
        /// \param[in] Flag of whether displaying the prior map trajectory.
        void setTrajFlag(int display_traj) {
            display_base_path = static_cast<bool>(display_traj);
        }

        /// \brief Acquire the desired keyframe pointer according to the inquiry index.
        /// \param[in] Inquiry index (int).
        /// \return the inquired keyframe pointer.
        /// \note std::shared_ptr only supported by C++ 11 standard or above.
        std::shared_ptr<KeyFrame> getKeyFrame(int index);


        nav_msgs::Path path[10];
        nav_msgs::Path base_path;
        sensor_msgs::PointCloud base_point_cloud;
        CameraPoseVisualization *posegraph_visualization;

        void savePoseGraph();

        void loadPoseGraph();

        void publish();

        Vector3d t_drift;
        double yaw_drift;
        Matrix3d r_drift;
        // world frame( base sequence or first sequence)<----> cur sequence frame
        Vector3d w_t_vio;
        Matrix3d w_r_vio;

        bool load_gps_info;
        Vector3d gps_0_trans; //gps_cur_2_old;
        Quaterniond gps_0_q;

        bool load_map;

    private:

        /// \brief Loop closure searching and matching algorithm.
        /// \param[in] keyframe
        /// \param[in] frame_index
        /// \return detected frame index of the keyframe database.
        int detectLoop(std::shared_ptr<KeyFrame> &keyframe, int frame_index);

        void addKeyFrameIntoImage(std::shared_ptr<KeyFrame> &keyframe);

        /// \brief 4D Optimization once detect a re-visited loop (Used with visual+IMU case).
        /// \param none.
        /// \return none.
        /// \note This function is internal function that \b MUST be called inside a while loop
        //     *      or standalone thead to keep running.
        void optimize4DoF();

        /// \brief 6D Optimization once detect a re-visited loop (Used with visual case).
        /// \param none.
        /// \return none.
        /// \note This function is internal function that \b MUST be called inside a while loop
        //     *      or standalone thead to keep running.
        void optimize6DoF();

        /**
         * @brief Updates trajectory after pose optimization updates
         */
        void updatePath();

        /**
         * @brief Keyframe database storing both loaded prior map and new added keyframes
         */
        std::list<std::shared_ptr<KeyFrame>> keyframelist;
        std::mutex m_keyframelist;
        std::mutex m_optimize_buf;
        std::mutex m_path;
        std::mutex m_drift;
        std::thread t_optimization;
        /**
         * @brief The std::queue of optimization candidate indexes.
         * @details Remaining elements will be cleared for new loop
         * because each optimization we only use the latest candidate.
         */
        std::queue<int> optimize_buf;

//        int count_;

        bool base_initialized_;

        /**
         * @brief The global index of keyframe.
         * @details Without prior map, the first index of new keyframe
         * will be 0. Otherwise the first new keyframe index will be the
         * size of prior map keyframe list.
         */
        int global_index;
        /**
         * @brief size of the prior map keyframe list.
         */
        int prior_max_index;

        /**
         * @brief The running sequence index. Default one is 1, prior map
         * counted as 0; if run new sequence, the sequence_cnt will add 1.
         */
        int sequence_cnt;

        vector<bool> sequence_loop;
        map<int, cv::Mat> image_pool;
        int earliest_loop_index;
        int earliest_neighbor_index;
        int base_sequence;
        bool use_imu;
        bool display_base_path;

        BriefDatabase db;
        BriefVocabulary *voc;

        ros::Publisher pub_pg_path;
        ros::Publisher pub_base_points;
        ros::Publisher pub_base_path;
        ros::Publisher pub_pose_graph;
        ros::Publisher pub_path[10];
    };

    template<typename T>
    inline
    void QuaternionInverse(const T q[4], T q_inverse[4]) {
        q_inverse[0] = q[0];
        q_inverse[1] = -q[1];
        q_inverse[2] = -q[2];
        q_inverse[3] = -q[3];
    };

    template<typename T>
    T NormalizeAngle(const T &angle_degrees) {
        if (angle_degrees > T(180.0))
            return angle_degrees - T(360.0);
        else if (angle_degrees < T(-180.0))
            return angle_degrees + T(360.0);
        else
            return angle_degrees;
    };

    class AngleLocalParameterization {
    public:

        template<typename T>
        bool operator()(const T *theta_radians, const T *delta_theta_radians,
                        T *theta_radians_plus_delta) const {
            *theta_radians_plus_delta =
                    NormalizeAngle(*theta_radians + *delta_theta_radians);

            return true;
        }

        static ceres::LocalParameterization *Create() {
            return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                    1, 1>);
        }
    };

    template<typename T>
    void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9]) {

        T y = yaw / T(180.0) * T(M_PI);
        T p = pitch / T(180.0) * T(M_PI);
        T r = roll / T(180.0) * T(M_PI);


        R[0] = cos(y) * cos(p);
        R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
        R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
        R[3] = sin(y) * cos(p);
        R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
        R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
        R[6] = -sin(p);
        R[7] = cos(p) * sin(r);
        R[8] = cos(p) * cos(r);
    }

    template<typename T>
    void RotationMatrixTranspose(const T R[9], T inv_R[9]) {
        inv_R[0] = R[0];
        inv_R[1] = R[3];
        inv_R[2] = R[6];
        inv_R[3] = R[1];
        inv_R[4] = R[4];
        inv_R[5] = R[7];
        inv_R[6] = R[2];
        inv_R[7] = R[5];
        inv_R[8] = R[8];
    }

    template<typename T>
    void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3]) {
        r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
        r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
        r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
    }

    struct FourDOFError {
        FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
                : t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i) {}

        template<typename T>
        bool operator()(const T *const yaw_i, const T *ti, const T *yaw_j, const T *tj, T *residuals) const {
            T t_w_ij[3];
            t_w_ij[0] = tj[0] - ti[0];
            t_w_ij[1] = tj[1] - ti[1];
            t_w_ij[2] = tj[2] - ti[2];

            // euler to rotation
            T w_R_i[9];
            YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
            // rotation transpose
            T i_R_w[9];
            RotationMatrixTranspose(w_R_i, i_R_w);
            // rotation matrix rotate point
            T t_i_ij[3];
            RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

            residuals[0] = (t_i_ij[0] - T(t_x));
            residuals[1] = (t_i_ij[1] - T(t_y));
            residuals[2] = (t_i_ij[2] - T(t_z));
            residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

            return true;
        }

        static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                           const double relative_yaw, const double pitch_i, const double roll_i) {
            return (new ceres::AutoDiffCostFunction<
                    FourDOFError, 4, 1, 3, 1, 3>(
                    new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
        }

        double t_x, t_y, t_z;
        double relative_yaw, pitch_i, roll_i;

    };

    struct FourDOFWeightError {
        FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
                : t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i) {
            weight = 1;
        }

        template<typename T>
        bool operator()(const T *const yaw_i, const T *ti, const T *yaw_j, const T *tj, T *residuals) const {
            T t_w_ij[3];
            t_w_ij[0] = tj[0] - ti[0];
            t_w_ij[1] = tj[1] - ti[1];
            t_w_ij[2] = tj[2] - ti[2];

            // euler to rotation
            T w_R_i[9];
            YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
            // rotation transpose
            T i_R_w[9];
            RotationMatrixTranspose(w_R_i, i_R_w);
            // rotation matrix rotate point
            T t_i_ij[3];
            RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

            residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
            residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
            residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
            residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);

            return true;
        }

        static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                           const double relative_yaw, const double pitch_i, const double roll_i) {
            return (new ceres::AutoDiffCostFunction<
                    FourDOFWeightError, 4, 1, 3, 1, 3>(
                    new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
        }

        double t_x, t_y, t_z;
        double relative_yaw, pitch_i, roll_i;
        double weight;

    };

    struct RelativeRTError {
        RelativeRTError(double t_x, double t_y, double t_z,
                        double q_w, double q_x, double q_y, double q_z,
                        double t_var, double q_var)
                : t_x(t_x), t_y(t_y), t_z(t_z),
                  q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
                  t_var(t_var), q_var(q_var) {}

        template<typename T>
        bool operator()(const T *const w_q_i, const T *ti, const T *w_q_j, const T *tj, T *residuals) const {
            T t_w_ij[3];
            t_w_ij[0] = tj[0] - ti[0];
            t_w_ij[1] = tj[1] - ti[1];
            t_w_ij[2] = tj[2] - ti[2];

            T i_q_w[4];
            QuaternionInverse(w_q_i, i_q_w);

            T t_i_ij[3];
            ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

            residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
            residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
            residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

            T relative_q[4];
            relative_q[0] = T(q_w);
            relative_q[1] = T(q_x);
            relative_q[2] = T(q_y);
            relative_q[3] = T(q_z);

            T q_i_j[4];
            ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

            T relative_q_inv[4];
            QuaternionInverse(relative_q, relative_q_inv);

            T error_q[4];
            ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);

            residuals[3] = T(2) * error_q[1] / T(q_var);
            residuals[4] = T(2) * error_q[2] / T(q_var);
            residuals[5] = T(2) * error_q[3] / T(q_var);

            return true;
        }

        static ceres::CostFunction *Create(const double t_x, const double t_y, const double t_z,
                                           const double q_w, const double q_x, const double q_y, const double q_z,
                                           const double t_var, const double q_var) {
            return (new ceres::AutoDiffCostFunction<
                    RelativeRTError, 6, 4, 3, 4, 3>(
                    new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
        }

        double t_x, t_y, t_z; //, t_norm;
        double q_w, q_x, q_y, q_z;
        double t_var, q_var;
    };
}