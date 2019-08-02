
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_DBoW2.h:

Program Listing for File DBoW2.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_DBoW2.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/DBoW2.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*
    * File: DBoW2.h
    * Date: November 2011
    * Author: Dorian Galvez-Lopez
    * Description: Generic include file for the DBoW2 classes and
    *   the specialized vocabularies and databases
    * License: see the LICENSE.txt file
    *
    */
   
   #ifndef __D_T_DBOW2__
   #define __D_T_DBOW2__
   
   namespace DBoW2 {}
   
   #include "TemplatedVocabulary.h"
   #include "TemplatedDatabase.h"
   #include "BowVector.h"
   #include "FeatureVector.h"
   #include "QueryResults.h"
   #include "FBrief.h"
   
   typedef DBoW2::TemplatedVocabulary<DBoW2::FBrief::TDescriptor, DBoW2::FBrief> 
     BriefVocabulary;
   
   typedef DBoW2::TemplatedDatabase<DBoW2::FBrief::TDescriptor, DBoW2::FBrief> 
     BriefDatabase;
   
   #endif
   
