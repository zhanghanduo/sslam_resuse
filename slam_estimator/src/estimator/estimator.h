/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file estimator.h
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief SLAM main process of SSLAM-pose_graph.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/ins_factor.h"
#include "../factor/gps_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"

using namespace noiseFactor;
typedef Matrix<double, 5, 1> Vector5d;
/**
 * @namespace slam_estimator
 */
namespace slam_estimator {
    class Estimator {
    public:
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    #endif /* DOXYGEN_SHOULD_SKIP_THIS */

        Estimator();

        ~Estimator();

        /**
         * @brief Set the covariance matrix of visual measurement reprojection error (ProjectionFactor)
         */
        void setParameter();

        /**
         * @brief Start the main thread of @ref processMeasurements().
         */
        void startProcessThread();

        // interface
        void initFirstPose(const Eigen::Vector3d &p, const Eigen::Matrix3d &r);


        void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);

        void inputINS(double t, const Vector3d &linearSpeed, const Quaterniond &angularRead, double height);

        void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);

        void inputGPS(double t, double x, double y, double z, double posAccuracy_x, double posAccuracy_y);

        /**
         * @brief Interface to input image data into @ref FeatureTracker class.
         * @param t
         * @param _img Left image data.
         * @param _img1 Right image data.
         * @param _disp (Optional) disparity map after stereo matching (refers to left image).
         * @param _mask (Optional) image mask for filtering out dynamic objects (refers to left image).
         */
        void
        inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat(),
                   const cv::Mat &_disp = cv::Mat(), const cv::Mat &_mask = cv::Mat());

        /**
         * @brief Process IMU data
         * @details IMU Preintegration, use the mean value integration method
         * to acquire current PQV as the initial value for optimization
         * @param t current time stamp
         * @param dt time interval
         * @param linear_acceleration
         * @param angular_velocity
         */
        void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

        /**
         * @brief Process INSPVA data
         * @details INSPVA data interpolation, according to the timestamp difference
         * between INS sensor data and image data.
         * @param t Current time stamp
         * @param dt Time interval between current and previous message time stamp.
         * @param linear_speed
         * @param angular_read
         * @param height_ current height value
         * @param last_ Flag of whether interpolation and slerp is needed. (last_ is true
         * means the INSPVA message is not in the image gap middle, so there is time difference)
         */
        void processINS(double t, double dt, const Vector3d &linear_speed,
                        const Quaterniond &angular_read, const double height_, const bool last_);

        void processGPS(double t, double dt, const Vector5d &gps_position, const bool last_);

        /**
         * @brief Process image feature data and solve pose estimation
         * @details Add feature points and calculates the tracking times and parallax to judge whether
         * it is a key frame. Also does extrinsic calibration if online calibration is required.
         * Then initialization is processed (VIO or VO) and sliding-window based optimization.
         * @param image Not the real "image" but the std::map containing all the feature properties
         * and camera ID as index.
         * @param header
         */
        void PoseSolver(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);

        /**
         * @brief The main loop function of processing IMU, INS and image data into the estimator system.
         */
        void processMeasurements();

        /**
         * @brief Clear and initialize all the state values of the sliding window
         */
        void clearState();

        /**
         * @brief visual structure initialization
         * @details to guarantee IMU is fully stimulated with motion.
         * @return bool true means initialization succeeds.
         */
        bool initialStructure();

        /**
         * @brief Joint initialization of VIO
         * @note Only used in IMU mode.
         * @return bool true means succeeds.
         */
        bool visualInitialAlign();

        /**
         * @brief Judge whether there is enough parallax (>30)
         * @param[out] relative_R Rotation matrix R between current frame and first frame
         * @param[out] relative_T Translation vector T between current frame and first frame
         * @param[out] l The corresponding frame that satisfies the
         * initialization condition in sliding window
         * @return bool true can be initialized; false otherwise.
         */
        bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);

        /**
         * @brief Process sliding window action.
         * @details If the second last frame is keyframe, then marginalizes the oldest frame and
         * turn the feature points (and IMU observations) into prior knowledge. If not keyframe,
         * abandons visual measurements (but keep the IMU observations to ensure continuity of
         * preintegration)
         */
        void slideWindow();

        /**
         * @brief Sliding window processes the observed frame count when
         * marginalizing the second last frame and replacing it with new key frame.
         */
        void slideWindowNew();

        /**
         * @brief Sliding window processes the observed frame count when
         * marginalizing out the oldest frame to keep efficiency.
         */
        void slideWindowOld();

        //***************************************************************************************
        //
        //! \brief  The main optimization including IMU factor, reprojection error factor and marginalization factor.
        //!
        //! \note   This function \b SHOULD be called after the initial guess of PNP estimation
        //! (FeatureManager.initFramePoseByPnP and FeatureManager.triangulate).
        //
        //***************************************************************************************
        void optimization();

        //***************************************************************************************
        //
        //! \brief  Copy the state vectors to parameter blocks for optimization process.
        //! \details The state vectors include Ps(translation), Rs(rotation), Vs, Bas, Bgs, tic, ric(external matrix),
        //! feature depth vector.
        //!
        //***************************************************************************************
        void vector2double();

        //***************************************************************************************
        //
        //! \brief  Copy the parameter blocks from optimization process to state vectors.
        //! \details The parameter blocks include para_Pose(7), para_SpeedBias(9), para_Ex_Pose(7), Para_Feature(1), para_Td.
        //!
        //***************************************************************************************
        void double2vector();

        /**
         * @brief Failure detection
         * @return true if there is a failure detection.
         */
        bool failureDetection();

        /**
         * @brief Put all the coming IMU messages into \c accVector and \c gyrVector
         * between the interval of @ref processMeasurements() sleep time.
         * @param t0 Last processing time stamp.
         * @param t1 Current time stamp.
         * @param[out] accVector Collection of linear acceleration data in small interval.
         * @param[out] gyrVector Collection of angular velocity data in small interval.
         * @return False if no IMU messages received in the duration; Otherwise true.
         */
        bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                            vector<pair<double, Eigen::Vector3d>> &gyrVector);

        /**
         * @brief Put all the coming INS messages into \c spdVector and \c angVector
         * between the interval of @ref processMeasurements() sleep time.
         * @param t0 Last processing time stamp.
         * @param t1 Current time stamp.
         * @param[out] spdVector Collection of linear velocity data in small interval.
         * @param[out] angVector Collection of angular data in small interval.
         * @return False if no INS messages received in the duration; Otherwise true.
         */
        bool getINSInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &spdVector,
                            vector<pair<double, Eigen::Quaterniond>> &angVector,
                            vector<pair<double, double>> &heightVector);


        bool getGPSInterval(double t0, double t1, vector<pair<double, Vector5d>> &gpsVector);

        /**
         * @brief Get the pose of current frame in world frame.
         * @param[out] T Inquired frame pose from current frame to world frame.
         */
        void getPoseInWorldFrame(Eigen::Matrix4d &T);

        /**
         * @brief Get the pose of inquired frame in world frame.
         * @param index Inquired frame.
         * @param[out] T Inquired frame pose from inquired frame to world frame.
         */
        void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);

        /**
         * @brief Predict next pose with assumption of constant velocity motion model
         */
        void predictPtsInNextFrame();

        /**
         * @brief Reject landmark feature outliers with reprojection error bounding.
         * @param[out] removeIndex std::set of indices to be removed.
         */
        void outliersRejection(set<int> &removeIndex);

        /**
         * @brief Calculate the 2D reprojection error given two frames \c i and \c j.
         * @param Ri Rotation matrix of \c ith body frame.
         * @param Pi Translation vector of \c ith body frame.
         * @param rici Extrinsic rotation matrix from camera frame to body frame, referring ith image.
         * @param tici Extrinsic tranlation matrix from camera frame to body frame, referring ith image.
         * @param Rj Rotation matrix of \c jth body frame.
         * @param Pj Translation vector of \c jth body frame.
         * @param ricj Extrinsic rotation matrix from camera frame to body frame, referring jth image.
         * @param ticj Extrinsic tranlation matrix from camera frame to body frame, referring jth image.
         * @param depth distance of the 3D point in the first observed frame.
         * @param uvi homogeneous 2D point in \c ith normalized camera coordinate frame
         * @param uvj homogeneous 2D point in \c jth normalized camera coordinate frame
         * @return The RMSE of 2D reprojection error on image coordinate.
         */
        static double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                 double depth, Vector3d &uvi, Vector3d &uvj);

        void updateLatestStates();

        void fastPredictIMU(double t, const Eigen::Vector3d &linear_acceleration,
                            const Eigen::Vector3d &angular_velocity);

        void fastPredictINS(double t, const Eigen::Vector3d &linear_speed,
                            const Eigen::Quaterniond &angular_read);

        bool IMUAvailable(double t);

        bool INSAvailable(double t);

        bool GPSAvailable(double t);

        void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

        void initFirstINSPose(vector<pair<double, Eigen::Vector3d>> &spdVector,
                              vector<pair<double, Eigen::Quaterniond>> &angVector,
                              vector<pair<double, double>> heightVector);

        /**
         * @enum SolverFlag estimator.h
         * @brief Current optimization state.
         */
        enum SolverFlag {
            INITIAL,
            NON_LINEAR
        };

        /**
         * @enum MarginalizationFlag estimator.h
         * @brief Marginalization types
         */
        enum MarginalizationFlag {
            /**
             * @brief Marginalize out the oldest frame.
             */
            MARGIN_OLD = 0,
            /**
             * @brief Marginalize the last frame out of the
             * sliding window and replace it with new key frame.
             */
            MARGIN_SECOND_NEW = 1
        };

        unsigned int count_;
        std::mutex mBuf;
        std::mutex mProcess;
        std::mutex mPropagate;
        queue<pair<double, Vector5d>> gpsBuf;
        queue<pair<double, Eigen::Vector3d>> accBuf;
        queue<pair<double, Eigen::Vector3d>> gyrBuf;
        queue<pair<double, Eigen::Vector3d>> spdBuf;
        queue<pair<double, Eigen::Quaterniond>> angBuf;
        queue<pair<double, double>> heightBuf;
        queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
        vector<pair<double, vector<double>>> gpsVec;
        double prevTime, curTime;
        bool openExEstimation;

//    std::thread trackThread;
        std::thread processThread;
        atomic<bool> processThread_swt;  // this goes in while(1) aka inf-while of processThread

        FeatureTracker featureTracker;

        SolverFlag solver_flag;
        MarginalizationFlag marginalization_flag;
        Vector3d g;

        Matrix3d ric[2];
        Vector3d tic[2];

        Vector3d Ps[(WINDOW_SIZE + 1)];
        Vector3d Vs[(WINDOW_SIZE + 1)];
        Matrix3d Rs[(WINDOW_SIZE + 1)];
        Vector3d Bas[(WINDOW_SIZE + 1)];
        Vector3d Bgs[(WINDOW_SIZE + 1)];
        double td;

        Matrix3d back_R0, last_R, last_R0;
        Vector3d back_P0, last_P, last_P0;
        double Headers[(WINDOW_SIZE + 1)];
        double last_time;

        IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
        Vector3d acc_0, gyr_0;
        Quaterniond ang_0;

        vector<double> dt_buf[(WINDOW_SIZE + 1)];
        vector<double> t_buf[(WINDOW_SIZE + 1)];
        vector<double> gt_buf[(WINDOW_SIZE + 1)];
		vector<bool> gps_status[(WINDOW_SIZE + 1)];
        vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
        vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];
        vector<Vector3d> linear_speed_buf[(WINDOW_SIZE + 1)];
        vector<Quaterniond> angular_read_buf[(WINDOW_SIZE + 1)];
        vector<Vector5d> gps_buf[(WINDOW_SIZE + 1)];
//        vector<double> height_read_buf[(WINDOW_SIZE + 1)];
        double sum_dt[(WINDOW_SIZE + 1)];

        int frame_count;
        int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
        int inputImageCnt;

        FeatureManager f_manager;
        MotionEstimator m_estimator;
        InitialEXRotation initial_ex_rotation;

        bool first_imu, first_gps;
        bool is_valid, is_key;
        bool failure_occur;

        vector<Vector3d> point_cloud;
        vector<Vector3d> margin_cloud;
        vector<Vector3d> key_poses;
        double initial_timestamp;

        double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
        double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
        double para_Feature[NUM_OF_F][SIZE_FEATURE];
        double para_Ex_Pose[2][SIZE_POSE];
        double para_Retrive_Pose[SIZE_POSE];
        double para_Td[1][1];

        int loop_window_index;

        // Customized ceres::CostFunction to describe last marginalization cost function.
        MarginalizationInfo *last_marginalization_info;
        vector<double *> last_marginalization_parameter_blocks;

        map<double, ImageFrame> all_image_frame;
        IntegrationBase *tmp_pre_integration;

        Eigen::Matrix3d cov_position;

        double latest_time;
        Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg,
                latest_acc_0, latest_gyr_0, last_vec_rev, latest_spd_0;
        Eigen::Quaterniond latest_Q, last_ang_rev;

        bool initFirstPoseFlag;
        bool initThreadFlag;
        bool gps_bad, gps_rec;
        Eigen::Vector2d offset;
//        bool init_kalman;
        bool initGPS;
//        bool initINS;
    };
}