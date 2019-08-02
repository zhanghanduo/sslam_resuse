
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_utility_CameraPoseVisualization.h:

Program Listing for File CameraPoseVisualization.h
==================================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_utility_CameraPoseVisualization.h>` (``/home/ugv/catkin_ws/src/sslam/slam_estimator/src/utility/CameraPoseVisualization.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   
   #pragma once
   
   #include <ros/ros.h>
   #include <std_msgs/ColorRGBA.h>
   #include <visualization_msgs/Marker.h>
   #include <visualization_msgs/MarkerArray.h>
   #include <Eigen/Dense>
   #include <Eigen/Geometry>
   
   class CameraPoseVisualization {
   public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW
       std::string m_marker_ns;
   
       CameraPoseVisualization(float r, float g, float b, float a);
   
       void setImageBoundaryColor(float r, float g, float b, float a = 1.0);
   
       void setOpticalCenterConnectorColor(float r, float g, float b, float a = 1.0);
   
       void setScale(double s);
   
       void setLineWidth(double width);
   
       void add_pose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
   
       void reset();
   
       void publish_by(ros::Publisher &pub, const std_msgs::Header &header);
   
       void add_edge(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1);
   
       void add_loopedge(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1);
   
   private:
       std::vector<visualization_msgs::Marker> m_markers;
       std_msgs::ColorRGBA m_image_boundary_color;
       std_msgs::ColorRGBA m_optical_center_connector_color;
       double m_scale;
       double m_line_width;
   
       static const Eigen::Vector3d imlt;
       static const Eigen::Vector3d imlb;
       static const Eigen::Vector3d imrt;
       static const Eigen::Vector3d imrb;
       static const Eigen::Vector3d oc;
       static const Eigen::Vector3d lt0;
       static const Eigen::Vector3d lt1;
       static const Eigen::Vector3d lt2;
   };
