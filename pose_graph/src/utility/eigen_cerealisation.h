//
// Created by hd on 18-10-11.
//

#ifndef PROJECT_EIGEN_CEREALISATION_H
#define PROJECT_EIGEN_CEREALISATION_H

#include "cereal/cereal.hpp"
#include "Eigen/Core"
#include <cstdint>

/**
 * @brief Serialisation of Eigen matrices for the serialisation
 * library cereal (http://uscilab.github.io/cereal/index.html).
 *
 * Contains serialisation for Eigen matrices to binary archives, i.e. matrices like
 * \c Eigen::MatrixXf, \c Eigen::Matrix4d, or \c Eigen::Vector3f.
 *
 * Todo: Add serialisation to and from JSON. Need to find out how to combine the two
 * variants of SFINAE that are used.
 */
namespace cereal {

/**
 * @brief Serialise an Eigen::Matrix using cereal.
 *
 * Note: Writes the binary data from Matrix::data(), so not sure what happens if a matrix ever has
 * non-contiguous data (if that can ever happen with Eigen).
 *
 * @param[in] ar The archive to serialise to.
 * @param[in] matrix The matrix to serialise.
 */
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

/**
 * @brief De-serialise an Eigen::Matrix using cereal.
 *
 * Reads the block of binary data back from a cereal archive into the Eigen::Matrix.
 *
 * @param[in] ar The archive to deserialise from.
 * @param[in] matrix The matrix to deserialise into.
 */
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
