
.. _program_listing_file__home_ugv_catkin_ws_src_sslam_pose_graph_src_utility_eigen_cerealisation.h:

Program Listing for File eigen_cerealisation.h
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file__home_ugv_catkin_ws_src_sslam_pose_graph_src_utility_eigen_cerealisation.h>` (``/home/ugv/catkin_ws/src/sslam/pose_graph/src/utility/eigen_cerealisation.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   //
   // Created by hd on 18-10-11.
   //
   
   #ifndef PROJECT_EIGEN_CEREALISATION_H
   #define PROJECT_EIGEN_CEREALISATION_H
   
   #include "cereal/cereal.hpp"
   #include "Eigen/Core"
   #include <cstdint>
   
   namespace cereal {
   
       template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
       inline
       typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
       save(Archive& ar, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
       {
           const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
           const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
           ar(rows);
           ar(cols);
           ar(binary_data(matrix.data(), rows * cols * sizeof(_Scalar)));
       };
   
       template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
       inline
       typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
       load(Archive& ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
       {
           std::int32_t rows;
           std::int32_t cols;
           ar(rows);
           ar(cols);
   
           matrix.resize(rows, cols);
   
           ar(binary_data(matrix.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
       };
   
   } /* namespace cereal */
   
   
   #endif //PROJECT_EIGEN_CEREALISATION_H
