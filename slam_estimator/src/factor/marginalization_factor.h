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
        std::vector<double *> parameter_blocks;
        std::vector<int> drop_set;

        double **raw_jacobians;
        std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
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

        int localSize(int size) const;

        int globalSize(int size) const;

        void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);

        void preMarginalize();

        void marginalize();

        std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

        std::vector<ResidualBlockInfo *> factors;
        // n number of residuals
        int m, n;
        std::unordered_map<long, int> parameter_block_size; //global size
        int sum_block_size;
        std::unordered_map<long, int> parameter_block_idx; //local size
        std::unordered_map<long, double *> parameter_block_data;

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
