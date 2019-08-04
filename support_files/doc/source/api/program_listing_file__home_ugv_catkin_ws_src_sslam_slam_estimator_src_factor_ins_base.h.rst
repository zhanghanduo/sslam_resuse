
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_factor_ins_base.h:

Program Listing for File ins_base.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_slam_estimator_src_factor_ins_base.h>` (``/home/ugv/catkin_ws/src/sslam/slam_estimator/src/factor/ins_base.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by hd on 22/7/19.
   //
   
   #ifndef SRC_INS_BASE_H
   #define SRC_INS_BASE_H
   
   #include "../utility/utility.h"
   #include "../estimator/parameters.h"
   
   #include <ceres/ceres.h>
   
   using namespace Eigen;
   
   class InterpolationBase {
   public:
       #ifndef DOXYGEN_SHOULD_SKIP_THIS
           EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   #endif /* DOXYGEN_SHOULD_SKIP_THIS */
   
       InterpolationBase() = delete;
   
       InterpolationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                        const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg){
   
       }
   
   
   
   };
   
   #endif //SRC_INS_BASE_H
