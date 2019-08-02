cmake_minimum_required(VERSION 2.8.3)
project(sslam)

if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(ColourBold  "${Esc}[1m")
    set(Red         "${Esc}[31m")
    set(Green       "${Esc}[32m")
    set(Yellow      "${Esc}[33m")
    set(Blue        "${Esc}[34m")
    set(Magenta     "${Esc}[35m")
    set(Cyan        "${Esc}[36m")
    set(White       "${Esc}[37m")
endif()

add_definitions(-std=c++14)
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
    sensor_msgs
    tf
    cv_bridge
    message_generation
    message_filters
    obstacle_msgs
    rds_msgs
#    camera_models
    image_transport)

#set(OpenCV_DIR /home/ugv/libraries/opencv-3.4.6/release/OpenCVConfig.cmake)
find_package(OpenCV 3.4.6 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})
#include(/home/ugv/libraries/opencv-3.4.6/release/OpenCVConfig.cmake)

find_package(Ceres REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
        #        INCLUDE_DIRS include
        LIBRARIES camera_models
        CATKIN_DEPENDS roscpp std_msgs
        #    DEPENDS system_lib
)

include_directories("camera_models/include")

message(STATUS "    OpenCV library status:")
message(STATUS "    version: ${Yellow} ${OpenCV_VERSION}${ColourReset}")
message(STATUS "    libraries: ${Yellow}${OpenCV_LIBS}${ColourReset}")
message(STATUS "    include path: ${Yellow}${OpenCV_INCLUDE_DIRS}${ColourReset}")
message(STATUS "    catkin libraries: ${Yellow}${catkin_LIBRARIES}${ColourReset}")

add_library(camera_models
        camera_models/src/chessboard/Chessboard.cc
        camera_models/src/calib/CameraCalibration.cc
        camera_models/src/camera_models/Camera.cc
        camera_models/src/camera_models/CameraFactory.cc
        camera_models/src/camera_models/CostFunctionFactory.cc
        camera_models/src/camera_models/PinholeCamera.cc
        camera_models/src/camera_models/PinholeFullCamera.cc
        camera_models/src/camera_models/CataCamera.cc
        camera_models/src/camera_models/EquidistantCamera.cc
        camera_models/src/camera_models/ScaramuzzaCamera.cc
        camera_models/src/sparse_graph/Transform.cc
        camera_models/src/gpl/gpl.cc
        camera_models/src/gpl/EigenQuaternionParameterization.cc)

#target_link_libraries(Calibrations ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

#add_subdirectory(camera_models)
add_subdirectory(slam_estimator)
add_subdirectory(pose_graph)

add_subdirectory(support_files/doc)

