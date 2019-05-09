/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 * 
 * This file is part of sslam.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * @file estimator.h
 * @brief This is the header file of main VO estimator.
 * @author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * @date 2019-03-03
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
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"


class Estimator
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Estimator();
    ~Estimator();

    void setParameter();

    void startProcessThread();

    // interface
    void initFirstPose(const Eigen::Vector3d& p, const Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat(), const cv::Mat &_mask = cv::Mat() );
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);
    void processMeasurements();

    // internal
    void clearState();

    bool initialStructure();

    bool visualInitialAlign();

    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);

    void slideWindow();

    void slideWindowNew();

    void slideWindowOld();

    //***************************************************************************************
    //
    //! \brief  The main optimization including IMU factor, reprojection error factor and marginalization factor.
    //!
    //! \param  none.
    //! \retval none.
    //!
    //! \note   This function \b SHOULD be called after the initial guess of PNP estimation
    //! (FeatureManager.initFramePoseByPnP and FeatureManager.triangulate).
    //
    //***************************************************************************************
    void optimization();

    //***************************************************************************************
    //
    //! \brief  Copy the state vectors to parameter blocks for optimization process.
    //! The state vectors include Ps(translation), Rs(rotation), Vs, Bas, Bgs, tic, ric(external matrix),
    //! feature depth vector.
    //!
    //! \param  none.
    //! \retval none.
    //!
    //
    //***************************************************************************************
    void vector2double();

    //***************************************************************************************
    //
    //! \brief  Copy the parameter blocks from optimization process to state vectors.
    //! The parameter blocks include para_Pose(7), para_SpeedBias(9), para_Ex_Pose(7), Para_Feature(1), para_Td.
    //!
    //! \param  none.
    //! \retval none.
    //!
    //
    //***************************************************************************************
    void double2vector();

    bool failureDetection();

    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void getGPS(double t, vector<double> &gps_msg);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    bool IMUAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    unsigned int count_;
    std::mutex mBuf;
    std::mutex mProcess;
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    vector<pair<double, vector<double>>> gpsVec;
    double prevTime, curTime;
    bool openExEstimation;

//    std::thread trackThread;
    std::thread processThread;
    atomic<bool> processThread_swt;  // this goes in while(1) aka inf-while of processThread

    FeatureTracker featureTracker;

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;

    Matrix3d ric[2];
    Vector3d tic[2];

    Matrix4d        GPs[(WINDOW_SIZE + 1)];
    Vector3d        Ps[(WINDOW_SIZE + 1)];
    Vector3d        Vs[(WINDOW_SIZE + 1)];
    Matrix3d        Rs[(WINDOW_SIZE + 1)];
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];
    double last_time;

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
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
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    Eigen::Matrix3d cov_position;
    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    bool initThreadFlag;
};
