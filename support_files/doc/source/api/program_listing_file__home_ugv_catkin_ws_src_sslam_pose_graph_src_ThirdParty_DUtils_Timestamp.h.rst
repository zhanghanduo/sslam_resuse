
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DUtils_Timestamp.h:

Program Listing for File Timestamp.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DUtils_Timestamp.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DUtils/Timestamp.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*
    * File: Timestamp.h
    * Author: Dorian Galvez-Lopez
    * Date: March 2009
    * Description: timestamping functions
    * License: see the LICENSE.txt file
    *
    */
   
   #ifndef __D_TIMESTAMP__
   #define __D_TIMESTAMP__
   
   #include <iostream>
   using namespace std;
   
   namespace DUtils {
   
   class Timestamp
   {
   public:
   
     enum tOptions
     {
       NONE = 0,
       CURRENT_TIME = 0x1,
       ZERO = 0x2
     };
     
   public:
     
       Timestamp(Timestamp::tOptions option = NONE);
       
       virtual ~Timestamp(void);
   
     bool empty() const;
   
       void setToCurrentTime();
   
       inline void setTime(unsigned long secs, unsigned long usecs){
           m_secs = secs;
           m_usecs = usecs;
       }
       
       inline void getTime(unsigned long &secs, unsigned long &usecs) const
       {
         secs = m_secs;
         usecs = m_usecs;
       }
   
       void setTime(const string &stime);
       
       void setTime(double s);
       
       double getFloatTime() const;
   
       string getStringTime() const;
   
       double operator- (const Timestamp &t) const;
   
       Timestamp plus(unsigned long s, unsigned long us) const;
   
     Timestamp minus(unsigned long s, unsigned long us) const;
   
     Timestamp& operator+= (double s);
     
     Timestamp& operator-= (double s);
   
       Timestamp operator+ (double s) const;
   
       Timestamp operator- (double s) const;
   
       bool operator> (const Timestamp &t) const;
   
       bool operator>= (const Timestamp &t) const;
   
       bool operator== (const Timestamp &t) const;
   
       bool operator< (const Timestamp &t) const;
   
       bool operator<= (const Timestamp &t) const;
   
     string Format(bool machine_friendly = false) const;
   
       static string Format(double s);
       
   
   protected:
       unsigned long m_secs;   // seconds
       unsigned long m_usecs;  // microseconds
   };
   
   }
   
   #endif
   
