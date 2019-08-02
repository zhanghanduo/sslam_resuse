
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_QueryResults.cpp:

Program Listing for File QueryResults.cpp
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_QueryResults.cpp>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/QueryResults.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #include <iostream>
   #include <fstream>
   #include "QueryResults.h"
   
   using namespace std;
   
   namespace DBoW2
   {
   
   // ---------------------------------------------------------------------------
   
   ostream & operator<<(ostream& os, const Result& ret )
   {
     os << "<EntryId: " << ret.Id << ", Score: " << ret.Score << ">";
     return os;
   }
   
   // ---------------------------------------------------------------------------
   
   ostream & operator<<(ostream& os, const QueryResults& ret )
   {
     if(ret.size() == 1)
       os << "1 result:" << endl;
     else
       os << ret.size() << " results:" << endl;
       
     QueryResults::const_iterator rit;
     for(rit = ret.begin(); rit != ret.end(); ++rit)
     {
       os << *rit;
       if(rit + 1 != ret.end()) os << endl;
     }
     return os;
   }
   
   // ---------------------------------------------------------------------------
   
   void QueryResults::saveM(const std::string &filename) const
   {
     fstream f(filename.c_str(), ios::out);
     
     QueryResults::const_iterator qit;
     for(qit = begin(); qit != end(); ++qit)
     {
       f << qit->Id << " " << qit->Score << endl;
     }
     
     f.close();
   }
   
   // ---------------------------------------------------------------------------
   
   } // namespace DBoW2
   
