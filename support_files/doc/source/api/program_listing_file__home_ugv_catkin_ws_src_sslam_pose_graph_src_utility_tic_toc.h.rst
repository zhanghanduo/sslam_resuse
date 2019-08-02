
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_utility_tic_toc.h:

Program Listing for File tic_toc.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_utility_tic_toc.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/utility/tic_toc.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*******************************************************
    * Copyright (C) 2019, Robotics Group, Nanyang Technology University
    *
    * \file tic_toc.h
    * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
    * \date Januarary 2017
    * \brief Time recording.
    *
    * Licensed under the GNU General Public License v3.0;
    * you may not use this file except in compliance with the License.
    *
    *******************************************************/
   
   #pragma once
   
   #include <ctime>
   #include <cstdlib>
   #include <chrono>
   
   namespace pose_graph {
       class TicToc {
       public:
           TicToc() {
               tic();
           }
   
           void tic() {
               start = std::chrono::system_clock::now();
           }
   
           double toc() {
               end = std::chrono::system_clock::now();
               std::chrono::duration<double> elapsed_seconds = end - start;
               return elapsed_seconds.count() * 1000;
           }
   
       private:
           std::chrono::time_point<std::chrono::system_clock> start, end;
       };
   }
