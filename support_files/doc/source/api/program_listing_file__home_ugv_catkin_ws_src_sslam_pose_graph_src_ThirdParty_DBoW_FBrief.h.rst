
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_FBrief.h:

Program Listing for File FBrief.h
=================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_FBrief.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/FBrief.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef __D_T_F_BRIEF__
   #define __D_T_F_BRIEF__
   
   #include <opencv2/opencv.hpp>
   #include <vector>
   #include <string>
   
   #include "FClass.h"
   #include "../DVision/DVision.h"
   
   namespace DBoW2 {
   
   class FBrief: protected FClass
   {
   public:
   
     typedef DVision::BRIEF::bitset TDescriptor;
     typedef const TDescriptor *pDescriptor;
   
     static void meanValue(const std::vector<pDescriptor> &descriptors, 
       TDescriptor &mean);
     
     static double distance(const TDescriptor &a, const TDescriptor &b);
     
     static std::string toString(const TDescriptor &a);
     
     static void fromString(TDescriptor &a, const std::string &s);
     
     static void toMat32F(const std::vector<TDescriptor> &descriptors, 
       cv::Mat &mat);
   
   };
   
   } // namespace DBoW2
   
   #endif
   
