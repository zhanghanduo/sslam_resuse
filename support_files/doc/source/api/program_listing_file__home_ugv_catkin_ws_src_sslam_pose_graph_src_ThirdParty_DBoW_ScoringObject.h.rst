
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_ScoringObject.h:

Program Listing for File ScoringObject.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DBoW_ScoringObject.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DBoW/ScoringObject.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef __D_T_SCORING_OBJECT__
   #define __D_T_SCORING_OBJECT__
   
   #include "BowVector.h"
   
   namespace DBoW2 {
   
   class GeneralScoring
   {
   public:
     virtual double score(const BowVector &v, const BowVector &w) const = 0;
   
     virtual bool mustNormalize(LNorm &norm) const = 0;
   
       static const double LOG_EPS; 
     // If you change the type of WordValue, make sure you change also the
       // epsilon value (this is needed by the KL method)
       
     virtual ~GeneralScoring() {} 
   };
   
   #define __SCORING_CLASS(NAME, MUSTNORMALIZE, NORM) \
     NAME: public GeneralScoring \
     { public: \
    \
       virtual double score(const BowVector &v, const BowVector &w) const; \
       \
    \
       virtual inline bool mustNormalize(LNorm &norm) const  \
         { norm = NORM; return MUSTNORMALIZE; } \
     }
     
   class __SCORING_CLASS(L1Scoring, true, L1);
   
   class __SCORING_CLASS(L2Scoring, true, L2);
   
   class __SCORING_CLASS(ChiSquareScoring, true, L1);
   
   class __SCORING_CLASS(KLScoring, true, L1);
   
   class __SCORING_CLASS(BhattacharyyaScoring, true, L1);
   
   class __SCORING_CLASS(DotProductScoring, false, L1);
   
   #undef __SCORING_CLASS
     
   } // namespace DBoW2
   
   #endif
   
