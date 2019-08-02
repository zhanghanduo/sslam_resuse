
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_VocabularyBinary.hpp:

Program Listing for File VocabularyBinary.hpp
=============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_VocabularyBinary.hpp>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/VocabularyBinary.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef VocabularyBinary_hpp
   #define VocabularyBinary_hpp
   
   #include <cstdint>
   #include <fstream>
   #include <string>
   
   namespace VINSLoop {
       
   struct Node {
       int32_t nodeId;
       int32_t parentId;
       double weight;
       uint64_t descriptor[4];
   };
   
   struct Word {
       int32_t nodeId;
       int32_t wordId;
   };
   
   struct Vocabulary {
       int32_t k;
       int32_t L;
       int32_t scoringType;
       int32_t weightingType;
       
       int32_t nNodes;
       int32_t nWords;
       
       Node* nodes;
       Word* words;
       
       Vocabulary();
       ~Vocabulary();
       
       void serialize(std::ofstream& stream);
       void deserialize(std::ifstream& stream);
       
       inline static size_t staticDataSize() {
           return sizeof(Vocabulary) - sizeof(Node*) - sizeof(Word*);
       }
   };
   
   }
   
   #endif /* VocabularyBinary_hpp */
