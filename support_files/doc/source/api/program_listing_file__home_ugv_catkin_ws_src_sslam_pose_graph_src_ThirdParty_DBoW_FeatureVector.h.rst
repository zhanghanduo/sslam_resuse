
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_FeatureVector.h:

Program Listing for File FeatureVector.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_FeatureVector.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/FeatureVector.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef __D_T_FEATURE_VECTOR__
   #define __D_T_FEATURE_VECTOR__
   
   #include "BowVector.h"
   #include "../../utility/cerealArchiver.h"
   #include <map>
   #include <vector>
   #include <iostream>
   
   namespace DBoW2 {
   
   class FeatureVector: 
     public std::map<NodeId, std::vector<unsigned int> >
   {
   public:
   
     FeatureVector();
     
     ~FeatureVector();
     
     void addFeature(NodeId id, unsigned int i_feature);
   
     friend std::ostream& operator<<(std::ostream &out, const FeatureVector &v);
   
       friend class cereal::access;
   
       template <class Archive>
       void serialize( Archive & ar )
       {
         ar (cereal::base_class<std::map<NodeId, std::vector<unsigned int> >>( this ));
       }
       
   };
   
   } // namespace DBoW2
   
   #endif
   
