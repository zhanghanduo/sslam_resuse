cmake_minimum_required(VERSION 2.8.3)
project(sslam)

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

catkin_package(
        #        INCLUDE_DIRS include
        LIBRARIES camera_models
        CATKIN_DEPENDS roscpp std_msgs
        #    DEPENDS system_lib
)

message(STATUS "    OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    catkin libraries: ${catkin_LIBRARIES}")

#add_subdirectory(camera_models)
add_subdirectory(slam_estimator)
add_subdirectory(pose_graph)

add_subdirectory(support_files/doc)

