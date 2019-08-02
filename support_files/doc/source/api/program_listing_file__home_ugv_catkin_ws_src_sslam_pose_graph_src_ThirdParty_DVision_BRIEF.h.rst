
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DVision_BRIEF.h:

Program Listing for File BRIEF.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DVision_BRIEF.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DVision/BRIEF.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   #ifndef __D_BRIEF__
   #define __D_BRIEF__
   
   #include <opencv2/opencv.hpp>
   #include <vector>
   #include <boost/dynamic_bitset.hpp>
   #include "../../utility/cerealArchiver.h"
   
   namespace DVision {
   
   class BRIEF
   {
   public:
   
     typedef boost::dynamic_bitset<> bitset;
   
     enum Type
     {
       RANDOM, // random pairs (Calonder's original version)
       RANDOM_CLOSE, // random but close pairs (used in GalvezIROS11)
     };
     
   public:
   
     BRIEF(int nbits = 256, int patch_size = 48, Type type = RANDOM_CLOSE);
     virtual ~BRIEF();
     
     inline int getDescriptorLengthInBits() const
     {
       return m_bit_length;
     }
     
     inline Type getType() const
     {
       return m_type;
     }
     
     inline int getPatchSize() const
     {
       return m_patch_size;
     }
     
     inline void operator() (const cv::Mat &image, 
       const std::vector<cv::KeyPoint> &points,
       std::vector<bitset> &descriptors,
       bool treat_image = true) const
     {
       compute(image, points, descriptors, treat_image);
     }
     
     void compute(const cv::Mat &image,
       const std::vector<cv::KeyPoint> &points,
       std::vector<bitset> &descriptors,
       bool treat_image = true) const;
     
     inline void exportPairs(std::vector<int> &x1, std::vector<int> &y1,
       std::vector<int> &x2, std::vector<int> &y2) const
     {
       x1 = m_x1;
       y1 = m_y1;
       x2 = m_x2;
       y2 = m_y2;
     }
     
     inline void importPairs(const std::vector<int> &x1, 
       const std::vector<int> &y1, const std::vector<int> &x2, 
       const std::vector<int> &y2)
     {
       m_x1 = x1;
       m_y1 = y1;
       m_x2 = x2;
       m_y2 = y2;
       m_bit_length = x1.size();
     }
     
     inline static int distance(const bitset &a, const bitset &b)
     {
       return (a^b).count();
     }
   
   protected:
   
     void generateTestPoints();
     
   protected:
   
     int m_bit_length;
   
     int m_patch_size;
     
     Type m_type;
   
     std::vector<int> m_x1, m_x2;
     std::vector<int> m_y1, m_y2;
   
       friend class cereal::access;
       template <class Archive>
       void serialize( Archive & ar )
       {
         ar (m_bit_length, m_patch_size);
       }
   
   };
   
   } // namespace DVision
   
   #endif
   
   
