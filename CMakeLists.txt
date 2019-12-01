cmake_minimum_required(VERSION 2.8.3)
project(sslam)

option(GPU_FEATURE "Use OpenCV CUDA to accelerate feature tracking. Only set to ON with CUDA supported OpenCV." ON)
option(SHOW_PROFILING "Profiling of each step." ON)


add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
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

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

find_package(OpenCV 3.3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
        #        INCLUDE_DIRS include
#        LIBRARIES camera_models
#        CATKIN_DEPENDS roscpp std_msgs
        #    DEPENDS system_lib
)

include_directories("camera_models/include")

message(STATUS "sslam    OpenCV library status:")
message(STATUS "sslam    version: ${OpenCV_VERSION}")
message(STATUS "sslam    libraries: ${OpenCV_LIBS}")
message(STATUS "sslam    include path: ${OpenCV_INCLUDE_DIRS}")

set(GPU_FEATURE OFF)
set(SHOW_PROFILING ON)

if( GPU_FEATURE )
add_definitions(-DGPU_FEATURE)
endif()
message(STATUS " GPU Feature status: ${GPU_FEATURE}")

if( SHOW_PROFILING )
    add_definitions(-DSHOW_PROFILING)
endif()
message(STATUS " Display Profiling status: ${SHOW_PROFILING}")

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

target_link_libraries(camera_models ${Boost_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_subdirectory(global_fusion)
add_subdirectory(slam_estimator)
add_subdirectory(pose_graph)
