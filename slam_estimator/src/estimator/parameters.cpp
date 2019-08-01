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

#include "parameters.h"
#include <ros/package.h>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
std::string INS_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int CUBICLE;
int USE_IMU;
int USE_INS;
int MULTIPLE_THREAD;
int ONLINE;
int USE_GPS;
int USE_GPU;
int USE_GPU_ACC_FLOW;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC, CUBICLE_TOPIC, GPS_TOPIC;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;
Eigen::Quaterniond gps_0_q;
Eigen::Vector3d gps_0_trans{0, 0, 0};
bool load_gps_info;

template<typename T>
T readParam(ros::NodeHandle &n, std::string name) {
    T ans;
    if (n.getParam(name, ans)) {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    } else {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(const std::string &config_file) {
    FILE *fh = fopen(config_file.c_str(), "r");
    if (fh == nullptr) {
        ROS_WARN("config_file doesn't exist; wrong config_file path");
        ROS_BREAK();
        return;
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    fsSettings["cubicle_topic"] >> CUBICLE_TOPIC;
    fsSettings["gps_topic"] >> GPS_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];
    ONLINE = fsSettings["online"];
    USE_GPS = fsSettings["gps_initial"];
    USE_GPU = fsSettings["use_gpu"];
    USE_GPU_ACC_FLOW = fsSettings["use_gpu_acc_flow"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if (USE_IMU) {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    USE_INS = fsSettings["ins"];
    printf("USE_INS: %d\n", USE_INS);
    if (USE_INS) {
        fsSettings["ins_topic"] >> INS_TOPIC;
        printf("INS_TOPIC: %s\n", INS_TOPIC.c_str());
//        ACC_N = fsSettings["acc_n"];
//        ACC_W = fsSettings["acc_w"];
//        GYR_N = fsSettings["gyr_n"];
//        GYR_W = fsSettings["gyr_w"];
//        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    OUTPUT_FOLDER = ros::package::getPath("sslam_estimator") + "/../output";
    RESULT_PATH = OUTPUT_FOLDER + "/vio.txt";
    std::cout << "result path " << RESULT_PATH << std::endl;
    std::ofstream fout(RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2) {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.emplace_back(Eigen::Matrix3d::Identity());
        TIC.emplace_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    } else {
        if (ESTIMATE_EXTRINSIC == 1) {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        else if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.emplace_back(T.block<3, 3>(0, 0));
        TIC.emplace_back(T.block<3, 1>(0, 3));
    }

    NUM_OF_CAM = fsSettings["num_of_cam"];
    CUBICLE = fsSettings["cubicle"];
    printf("camera number %d\n", NUM_OF_CAM);

    if (NUM_OF_CAM != 1 && NUM_OF_CAM != 2) {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);

    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if (NUM_OF_CAM == 2) {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib;
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);

        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.emplace_back(T.block<3, 3>(0, 0));
        TIC.emplace_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if (!USE_IMU && !USE_INS) {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu or ins, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();
}
