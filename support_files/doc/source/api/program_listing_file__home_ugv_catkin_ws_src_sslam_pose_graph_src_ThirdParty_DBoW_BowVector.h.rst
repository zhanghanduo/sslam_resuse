
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_BowVector.h:

Program Listing for File BowVector.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_BowVector.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/BowVector.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef __D_T_BOW_VECTOR__
   #define __D_T_BOW_VECTOR__
   
   #include <iostream>
   #include <map>
   #include <vector>
   #include "../../utility/cerealArchiver.h"
   
   namespace DBoW2 {
   
   typedef unsigned int WordId;
   
   typedef double WordValue;
   
   typedef unsigned int NodeId;
   
   enum LNorm
   {
     L1,
     L2
   };
   
   enum WeightingType
   {
     TF_IDF,
     TF,
     IDF,
     BINARY
   };
   
   enum ScoringType
   {
     L1_NORM,
     L2_NORM,
     CHI_SQUARE,
     KL,
     BHATTACHARYYA,
     DOT_PRODUCT
   };
   
   class BowVector: 
       public std::map<WordId, WordValue>
   {
   public:
   
       BowVector();
   
       ~BowVector();
       
       void addWeight(WordId id, WordValue v);
       
       void addIfNotExist(WordId id, WordValue v);
   
       void normalize(LNorm norm_type);
       
       friend std::ostream& operator<<(std::ostream &out, const BowVector &v);
       
       void saveM(const std::string &filename, size_t W) const;
   
       friend class cereal::access;
   
       template <class Archive>
       void serialize( Archive & ar )
       {
           ar (cereal::base_class<std::map<WordId, WordValue>>( this ));
       }
   };
   
   } // namespace DBoW2
   
   #endif
