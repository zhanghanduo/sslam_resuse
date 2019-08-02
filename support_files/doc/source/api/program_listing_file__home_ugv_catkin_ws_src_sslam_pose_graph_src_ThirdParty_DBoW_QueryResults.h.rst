
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_QueryResults.h:

Program Listing for File QueryResults.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_QueryResults.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/QueryResults.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef __D_T_QUERY_RESULTS__
   #define __D_T_QUERY_RESULTS__
   
   #include <vector>
   
   namespace DBoW2 {
   
   typedef unsigned int EntryId;
   
   class Result
   {
   public:
     
     EntryId Id;
     
     double Score;
     
     int nWords; // words in common
     // !!! this is filled only by Bhatt score!
     // (and for BCMatching, BCThresholding then)
     
     double bhatScore, chiScore;
     
     // only done by ChiSq and BCThresholding 
     double sumCommonVi;
     double sumCommonWi;
     double expectedChiScore;
   
     inline Result(){}
     
     inline Result(EntryId _id, double _score): Id(_id), Score(_score){}
   
     inline bool operator<(const Result &r) const
     {
       return this->Score < r.Score;
     }
   
     inline bool operator>(const Result &r) const
     {
       return this->Score > r.Score;
     }
   
     inline bool operator==(EntryId id) const
     {
       return this->Id == id;
     }
     
     inline bool operator<(double s) const
     {
       return this->Score < s;
     }
     
     inline bool operator>(double s) const
     {
       return this->Score > s;
     }
     
     static inline bool gt(const Result &a, const Result &b)
     {
       return a.Score > b.Score;
     }
     
     inline static bool ge(const Result &a, const Result &b)
     {
       return a.Score > b.Score;
     }
     
     static inline bool geq(const Result &a, const Result &b)
     {
       return a.Score >= b.Score;
     }
     
     static inline bool geqv(const Result &a, double s)
     {
       return a.Score >= s;
     }
     
     
     static inline bool ltId(const Result &a, const Result &b)
     {
       return a.Id < b.Id;
     }
     
     friend std::ostream & operator<<(std::ostream& os, const Result& ret );
   };
   
   class QueryResults: public std::vector<Result>
   {
   public:
   
     inline void scaleScores(double factor);
     
     friend std::ostream & operator<<(std::ostream& os, const QueryResults& ret );
     
     void saveM(const std::string &filename) const;
     
   };
   
   // --------------------------------------------------------------------------
   
   inline void QueryResults::scaleScores(double factor)
   {
     for(QueryResults::iterator qit = begin(); qit != end(); ++qit) 
       qit->Score *= factor;
   }
   
   // --------------------------------------------------------------------------
   
   } // namespace TemplatedBoW
     
   #endif
   
