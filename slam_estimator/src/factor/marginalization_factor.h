/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * \file marginalizatioin_factor.h
 * \author Zhang Handuo (hzhang032@e.ntu.edu.sg)
 * \date Januarary 2017
 * \brief Config parameters read from external config files.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <utility>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 8;
/**
 * @namespace noiseFactor
 */
namespace noiseFactor {

    /**
     * @struct ResidualBlockInfo
     * @brief Define the data and error of one observation
     * to be marginalized out using Shcur Complement algorithm.
     */
    struct ResidualBlockInfo {
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    #endif /* DOXYGEN_SHOULD_SKIP_THIS */


        ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function,
                          std::vector<double *> _parameter_blocks, std::vector<int> _drop_set);

        void Evaluate();

        ceres::CostFunction *cost_function;
        ceres::LossFunction *loss_function;
        // The data of variables to be optimized.
        std::vector<double *> parameter_blocks;
	    // Drop_set is the IDs of variables to be marginalized.
        std::vector<int> drop_set;

        double **raw_jacobians;
        std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
        // Residuals, IMU: 15x1, visual: 2x1
        Eigen::VectorXd residuals;

        int localSize(int size) {
            return size == 7 ? 6 : size;
        }
    };

    struct ThreadsStruct {
#ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */
        std::vector<ResidualBlockInfo *> sub_factors;
        // Hessian matrix A.
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
        std::unordered_map<long, int> parameter_block_size; //global size
        std::unordered_map<long, int> parameter_block_idx; //local size
    };

    /**
     * @class MarginalizationInfo
     * @brief Contains all the @ResidualBlockInfo to be marginalized.
     */
    class MarginalizationInfo {
    public:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

        MarginalizationInfo() { valid = true; };

        ~MarginalizationInfo();

        // Add residual block information (variables to be optimized and marginalized).
        void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);

        // Calculate Jacobian of each residual, and update "parameter_block_data"
        void preMarginalize();

        // pos: dimension of all variables.
        void marginalize();

        std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

        // All the observable factors (in form of residual block information).
        std::vector<ResidualBlockInfo *> factors;

        // pos: size of all variables (local size).
        // n: number of remaining variables to be optimized (local size).
        // m: number of variables to be marginalized (local size).

        int m, n;

        // The memory address of variables to be optimized.
        // <long, int> where first element is address of variable.
        // second element is the number of outputs (residuals) in "parameter_block_sizes_" in ceres::cost_function

        // "parameter_block_sizes_":
        // parameters in cost_function is an array of pointers to arrays containing the
        // various parameter blocks. parameters has the same number of
        // elements as parameter_block_sizes_.  Parameter blocks are in the
        // same order as parameter_block_sizes_.i.e.,
        //
	    //   parameters_[i] = double[parameter_block_sizes_[i]]

        std::unordered_map<long, int> parameter_block_size; //global size
	    // Corresponding data (double*) of "paramter_block_size".
	    std::unordered_map<long, double *> parameter_block_data;

        int sum_block_size;
	    // Ids in "parameter_block_size", the memory address of variables to be marginalized.
        std::unordered_map<long, int> parameter_block_idx; //local size


	    // Example: frame 0 observed k=68 landmarks, then:
	    // parameter_block_size size: 2(V0,V1) + 11(P0,...,P10) + 1 (Tbc) + 1(tb) + 68 = 83.
	    // pos: 9x2 + 6x11 + 6 + 1 + 68 = 159.
	    // parameter_block_idx size: 1(P0) + 1(V0) + 68 = 70.
	    // m: 6 + 9 + 68 = 83.
	    // n = pos - m = 159 - 83 = 76.

        std::vector<int> keep_block_size; //global size
        std::vector<int> keep_block_idx;  //local size
        std::vector<double *> keep_block_data;

        Eigen::MatrixXd linearized_jacobians;
        Eigen::VectorXd linearized_residuals;
        const double eps = 1e-8;
        bool valid;
    };

    /**
     * @class MarginalizationFactor
     * @brief Define the cost function of marginalization information @ref MarginalizationInfo.
     */
    class MarginalizationFactor : public ceres::CostFunction {
    public:
        MarginalizationFactor(MarginalizationInfo *_marginalization_info);

        bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;

        MarginalizationInfo *marginalization_info;
    };
}
