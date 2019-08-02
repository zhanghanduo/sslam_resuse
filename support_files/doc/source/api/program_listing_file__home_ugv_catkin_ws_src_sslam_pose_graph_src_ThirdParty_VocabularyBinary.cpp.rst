
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_VocabularyBinary.cpp:

Program Listing for File VocabularyBinary.cpp
=============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_VocabularyBinary.cpp>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/VocabularyBinary.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "VocabularyBinary.hpp"
   #include <opencv2/core/core.hpp>
   using namespace std;
   
   VINSLoop::Vocabulary::Vocabulary()
   : nNodes(0), nodes(nullptr), nWords(0), words(nullptr) {
   }
   
   VINSLoop::Vocabulary::~Vocabulary() {
       if (nodes != nullptr) {
           delete [] nodes;
           nodes = nullptr;
       }
       
       if (words != nullptr) {
           delete [] words;
           words = nullptr;
       }
   }
       
   void VINSLoop::Vocabulary::serialize(ofstream& stream) {
       stream.write((const char *)this, staticDataSize());
       stream.write((const char *)nodes, sizeof(Node) * nNodes);
       stream.write((const char *)words, sizeof(Word) * nWords);
   }
       
   void VINSLoop::Vocabulary::deserialize(ifstream& stream) {
       stream.read((char *)this, staticDataSize());
       
       nodes = new Node[nNodes];
       stream.read((char *)nodes, sizeof(Node) * nNodes);
       
       words = new Word[nWords];
       stream.read((char *)words, sizeof(Word) * nWords);
   }
