cmake_minimum_required(VERSION 2.8.3)
project(sslam_estimator)

add_library(sslam_lib
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp)
target_link_libraries(sslam_lib ${OpenCV_LIBS} ${catkin_LIBRARIES}  ${CERES_LIBRARIES} camera_models)

add_executable(sslam_estimator_node src/rosNodeTest.cpp)
target_link_libraries(sslam_estimator_node sslam_lib)

add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
target_link_libraries(kitti_odom_test sslam_lib)

add_executable(kitti_raw_test src/KITTIRawTest.cpp)
target_link_libraries(kitti_raw_test sslam_lib)

add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
target_link_libraries(kitti_gps_test sslam_lib)

