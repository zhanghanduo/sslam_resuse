cmake_minimum_required(VERSION 2.8.3)
project(sslam_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(OPTIMIZATION_FLAGS "-O4 -Wall")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${OPTIMIZATION_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${OPTIMIZATION_FLAGS} -g")
## Require C++14 support, since we use specific features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    message_generation
    message_filters
    obstacle_msgs
    camera_models
    image_transport)

find_package(OpenCV 3.4 REQUIRED)
#include(/home/hd/softwares/opencv-3.4.5/release/OpenCVConfig.cmake)

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    catkin libraries: ${catkin_LIBRARIES}")

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
target_link_libraries(sslam_lib ${OpenCV_LIBS} ${catkin_LIBRARIES}  ${CERES_LIBRARIES})


add_executable(sslam_estimator_node src/rosNodeTest.cpp)
target_link_libraries(sslam_estimator_node sslam_lib)

add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
target_link_libraries(kitti_odom_test sslam_lib)

add_executable(kitti_raw_test src/KITTIRawTest.cpp)
target_link_libraries(kitti_raw_test sslam_lib)

add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
target_link_libraries(kitti_gps_test sslam_lib)

