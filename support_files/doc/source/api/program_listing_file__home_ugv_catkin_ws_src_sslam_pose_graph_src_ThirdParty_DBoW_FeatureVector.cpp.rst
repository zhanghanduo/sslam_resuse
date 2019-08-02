
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_FeatureVector.cpp:

Program Listing for File FeatureVector.cpp
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_FeatureVector.cpp>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/FeatureVector.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #include "FeatureVector.h"
   #include <map>
   #include <vector>
   #include <iostream>
   
   namespace DBoW2 {
   
   // ---------------------------------------------------------------------------
   
   FeatureVector::FeatureVector(void)
   {
   }
   
   // ---------------------------------------------------------------------------
   
   FeatureVector::~FeatureVector(void)
   {
   }
   
   // ---------------------------------------------------------------------------
   
   void FeatureVector::addFeature(NodeId id, unsigned int i_feature)
   {
     FeatureVector::iterator vit = this->lower_bound(id);
     
     if(vit != this->end() && vit->first == id)
     {
       vit->second.push_back(i_feature);
     }
     else
     {
       vit = this->insert(vit, FeatureVector::value_type(id, 
         std::vector<unsigned int>() ));
       vit->second.push_back(i_feature);
     }
   }
   
   // ---------------------------------------------------------------------------
   
   std::ostream& operator<<(std::ostream &out, 
     const FeatureVector &v)
   {
     if(!v.empty())
     {
       FeatureVector::const_iterator vit = v.begin();
       
       const std::vector<unsigned int>* f = &vit->second;
   
       out << "<" << vit->first << ": [";
       if(!f->empty()) out << (*f)[0];
       for(unsigned int i = 1; i < f->size(); ++i)
       {
         out << ", " << (*f)[i];
       }
       out << "]>";
       
       for(++vit; vit != v.end(); ++vit)
       {
         f = &vit->second;
         
         out << ", <" << vit->first << ": [";
         if(!f->empty()) out << (*f)[0];
         for(unsigned int i = 1; i < f->size(); ++i)
         {
           out << ", " << (*f)[i];
         }
         out << "]>";
       }
     }
     
     return out;  
   }
   
   // ---------------------------------------------------------------------------
   
   } // namespace DBoW2
