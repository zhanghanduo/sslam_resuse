
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DUtils_DException.h:

Program Listing for File DException.h
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_ThirdParty_DUtils_DException.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/ThirdParty/DUtils/DException.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*  
    * File: DException.h
    * Project: DUtils library
    * Author: Dorian Galvez-Lopez
    * Date: October 6, 2009
    * Description: general exception of the library
    * License: see the LICENSE.txt file
    *
    */
   
   #pragma once
   
   #ifndef __D_EXCEPTION__
   #define __D_EXCEPTION__
   
   #include <stdexcept>
   #include <string>
   using namespace std;
   
   namespace DUtils {
   
   class DException :
       public exception
   {
   public:
       DException(void) throw(): m_message("DUtils exception"){}
   
       DException(const char *msg) throw(): m_message(msg){}
       
       DException(const string &msg) throw(): m_message(msg){}
   
       virtual ~DException(void) throw(){}
   
       virtual const char* what() const throw()
       {
           return m_message.c_str();
       }
   
   protected:
       string m_message;
   };
   
   }
   
   #endif
   
