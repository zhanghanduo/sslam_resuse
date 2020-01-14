cmake_minimum_required(VERSION 2.8.3)
#project(pose_graph)

find_package(Cereal)

include_directories(
        ${PROJECT_SOURCE_DIR}/src/utility
        ${CEREAL_INCLUDE_DIR})

#catkin_package()

set(NODELET_SRC
        src/keyframe.cpp
        src/utility/CameraPoseVisualization.cpp
        src/utility/kalman.cpp
        src/ThirdParty/DBoW/BowVector.cpp
        src/ThirdParty/DBoW/FBrief.cpp
        src/ThirdParty/DBoW/FeatureVector.cpp
        src/ThirdParty/DBoW/QueryResults.cpp
        src/ThirdParty/DBoW/ScoringObject.cpp
        src/ThirdParty/DUtils/Random.cpp
        src/ThirdParty/DUtils/Timestamp.cpp
        src/ThirdParty/DVision/BRIEF.cpp
        src/ThirdParty/VocabularyBinary.cpp
        )

add_executable(pose_graph_node
        src/pose_graph_node.cpp src/pose_graph.cpp ${NODELET_SRC} )

target_link_libraries(pose_graph_node
        ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} camera_models)

add_executable(gps_graph_node
        src/gps_graph_node.cpp src/gps_graph.cpp ${NODELET_SRC} )

target_link_libraries(gps_graph_node
        ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} camera_models)