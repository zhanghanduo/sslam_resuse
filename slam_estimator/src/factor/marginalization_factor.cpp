/*******************************************************
 * Copyright (C) 2019, Robotics Group, Nanyang Technology University
 *
 * This file is part of sslam.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Zhang Handuo (hzhang032@e.ntu.edu.sg)
 *******************************************************/

#include "marginalization_factor.h"

#include <utility>
/**
 * @namespace noiseFactor
 */
namespace noiseFactor {

	ResidualBlockInfo::ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function,
	                                     std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
			: cost_function(_cost_function), loss_function(_loss_function),
			  parameter_blocks(std::move(_parameter_blocks)), drop_set(std::move(_drop_set)) {}

	void ResidualBlockInfo::Evaluate() {
		// No of outputs:
		// num_residuals_
		residuals.resize(cost_function->num_residuals());

		// No of inputs:
		// parameter_block_sizes_
		std::vector<int> block_sizes = cost_function->parameter_block_sizes();

		// jacobians is an array of size parameter_block_sizes_ containing
		// pointers to storage for jacobian blocks corresponding to each
		// parameter block. Jacobian blocks are in the same order as
		// parameter_block_sizes, i.e. jacobians[i], is an
		// array that contains num_residuals_* parameter_block_sizes_[i]
		// elements. Each jacobian block is stored in row-major order, i.e.,
		//
		//   jacobians[i][r*parameter_block_size_[i] + c] =
		//                              d residual[r] / d parameters[i][c]
		raw_jacobians = new double *[block_sizes.size()];

		jacobians.resize(block_sizes.size());

		for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
			jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
			raw_jacobians[i] = jacobians[i].data();
			//dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
		}
		cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

		if (loss_function) {
			double residual_scaling_, alpha_sq_norm_;

			double sq_norm, rho[3];

			sq_norm = residuals.squaredNorm();
			loss_function->Evaluate(sq_norm, rho);
			//printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

			double sqrt_rho1_ = sqrt(rho[1]);

			if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
				residual_scaling_ = sqrt_rho1_;
				alpha_sq_norm_ = 0.0;
			} else {
				const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
				const double alpha = 1.0 - sqrt(D);
				residual_scaling_ = sqrt_rho1_ / (1 - alpha);
				alpha_sq_norm_ = alpha / sq_norm;
			}

			for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
				jacobians[i] =
						sqrt_rho1_ *
						(jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
			}

			residuals *= residual_scaling_;
		}
	}


    MarginalizationFactor::MarginalizationFactor(MarginalizationInfo *_marginalization_info)
            : marginalization_info(_marginalization_info) {
//        int cnt = 0;
        for (auto it : marginalization_info->keep_block_size) {
            mutable_parameter_block_sizes()->push_back(it);
//            cnt += it;
        }
        //printf("residual size: %d, %d\n", cnt, n);
        set_num_residuals(marginalization_info->n);
    }

    MarginalizationInfo::~MarginalizationInfo() {
        //ROS_WARN("release marginalization info");
        for (auto &it : parameter_block_data)
            delete[] it.second;

        for (auto &factor : factors) {
            delete[] factor->raw_jacobians;
            delete factor->cost_function;
            delete factor;
        }
    }

	void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info) {
        factors.emplace_back(residual_block_info);

        std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
        std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

        for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++) {
        	// The address of each variable of newly added residual block information.
            double *addr = parameter_blocks[i];
            int size = parameter_block_sizes[i];
            parameter_block_size[reinterpret_cast<long>(addr)] = size;
        }

        for (int i : residual_block_info->drop_set) {
            double *addr = parameter_blocks[i];
            // As later in "Marginalize" we will reassign local IDs,
            // we temporarily only initialize them all 0.
            parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
        }
    }

    // Calculate each residual and its Jacobian.
    // Also update the parameter_block_data to add new residuals.
    void MarginalizationInfo::preMarginalize() {
        for (auto it : factors) {
        	// For each variable (residual block) evaluate the residual with Jacobian.
            it->Evaluate();

            std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
            // For each parameter block element in the residual block,
            // Copy the data of new added residual blocks to marginalization class.
            for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
                long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
                int size = block_sizes[i];
                if (parameter_block_data.find(addr) == parameter_block_data.end()) {
                    auto *data = new double[size];
                    memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
                    parameter_block_data[addr] = data;
                }
            }
        }
    }

	// It is desirable to choose a parameterization for the block
	// itself to remove the null directions of the cost. More generally,
	// if x lies on a manifold of a smaller dimension than the ambient
	// space that it is embedded in, then it is numerically and computationally
	// more effective to optimize it using a parameterization that lives in
	// the tangent space of that manifold at each point.
    int localSize(int size)  {
        return size == 7 ? 6 : size;
    }

//    int globalSize(int size)  {
//        return size == 6 ? 7 : size;
//    }

    // Construct Hessian matrix A and the least square form Ax=b from Jacobian matrix in each thread.
    void *ThreadsConstructA(void *threadsstruct) {
        ThreadsStruct *p = ((ThreadsStruct *) threadsstruct);
        for (auto it : p->sub_factors) {
            for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
                int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
                int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
                size_i = localSize(size_i);
                Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
                for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++) {
                    int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                    int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                    size_j = localSize(size_j);
                    Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                    if (i == j)
                        p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    else {
                        p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                        p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                    }
                }
                p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
            }
        }
        return threadsstruct;
    }

    // Construct prior Schur Complement AX=b to calculate residuals with Jacobian
    void MarginalizationInfo::marginalize() {
		// pos represents size of all variables (locally, set 0 each time starts).
        int pos = 0;
        // 1. For variables waiting to be marginalized.
        for (auto &it : parameter_block_idx) {
            // Assign the space for marginalization variables
            // it.first is the memory address (long)
            // it.second is the localSize (for pose local size is 6)
            it.second = pos;
            pos += localSize(parameter_block_size[it.first]);
        }

        m = pos;

        // 2. For variables remaining to be optimized.
        for (const auto &it : parameter_block_size) {
            if (parameter_block_idx.find(it.first) == parameter_block_idx.end()) {
                parameter_block_idx[it.first] = pos;
                pos += localSize(it.second);
            }
        }

        // n is the remaining variables to be optimized.
        n = pos - m;
        //ROS_INFO("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());
        if (m == 0) {
            valid = false;
            printf("unstable tracking...\n");
            return;
        }

//    TicToc t_summing;
        Eigen::MatrixXd A(pos, pos);
        Eigen::VectorXd b(pos);
        A.setZero();
        b.setZero();
        /*
        for (auto it : factors)
        {
            for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
            {
                int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
                int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
                Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
                for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
                {
                    int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                    int size_j = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
                    Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                    if (i == j)
                        A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    else
                    {
                        A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                        A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
                    }
                }
                b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
            }
        }
        ROS_INFO("summing up costs %f ms", t_summing.toc());
        */
        //multi thread


        TicToc t_thread_summing;
        pthread_t tids[NUM_THREADS];
        ThreadsStruct threadsstruct[NUM_THREADS];
        int i = 0;
        // Assign the factors into different threads.
        for (auto it : factors) {
            threadsstruct[i].sub_factors.push_back(it);
            i++;
            i = i % NUM_THREADS;
        }
        for (int i2 = 0; i2 < NUM_THREADS; i2++) {
//            TicToc zero_matrix;
            threadsstruct[i2].A = Eigen::MatrixXd::Zero(pos, pos);
            threadsstruct[i2].b = Eigen::VectorXd::Zero(pos);
            threadsstruct[i2].parameter_block_size = parameter_block_size;
            threadsstruct[i2].parameter_block_idx = parameter_block_idx;
            int ret = pthread_create(&tids[i2], nullptr, ThreadsConstructA, (void *) &(threadsstruct[i2]));
            if (ret != 0) {
                ROS_WARN("pthread_create error");
                ROS_BREAK();
            }
        }
        for (int i3 = NUM_THREADS - 1; i3 >= 0; i3--) {
            pthread_join(tids[i3], nullptr);
            A += threadsstruct[i3].A;
            b += threadsstruct[i3].b;
        }
//    ROS_INFO("thread summing up costs %f ms", t_thread_summing.toc());
        //ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());


        Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);

        //ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());

        Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps)
                                                                                .select(saes.eigenvalues().array().inverse(),
                                                                                        0)).asDiagonal() *
                                  saes.eigenvectors().transpose();
        //printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

        // Schur Complement.
        Eigen::VectorXd bmm = b.segment(0, m);
        Eigen::MatrixXd Amr = A.block(0, m, m, n);
        Eigen::MatrixXd Arm = A.block(m, 0, n, m);
        Eigen::MatrixXd Arr = A.block(m, m, n, n);
        Eigen::VectorXd brr = b.segment(m, n);
        A = Arr - Arm * Amm_inv * Amr;
        b = brr - Arm * Amm_inv * bmm;

        // A here is Hermitian matrix, so use "SelfAdjointEigenSolver" to decomposite A with SVD.
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
        Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv = Eigen::VectorXd(
                (saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

        Eigen::VectorXd S_sqrt = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

        // J0 = sqrt(S) * V_t, corresponding linearized jacobian matrix
        linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        // b0 = sqrt(S^(-1)) * V_t * b, linearized residual.
        linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;

        //std::cout << A << std::endl
        //          << std::endl;
        //std::cout << linearized_jacobians << std::endl;
        //printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
        //      (linearized_jacobians.transpose() * linearized_residuals - b).sum());
    }

    std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double *> &addr_shift) {
        std::vector<double *> keep_block_addr;
        keep_block_size.clear();
        keep_block_idx.clear();
        keep_block_data.clear();

        for (const auto &it : parameter_block_idx) {
            if (it.second >= m) {
                keep_block_size.push_back(parameter_block_size[it.first]);
                keep_block_idx.push_back(parameter_block_idx[it.first]);
                keep_block_data.push_back(parameter_block_data[it.first]);
                keep_block_addr.push_back(addr_shift[it.first]);
            }
        }
        sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

        return keep_block_addr;
    }

    bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        //printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
        //for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
        //{
        //    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
        //    //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
        //printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
        //printf("residual %x\n", reinterpret_cast<long>(residuals));
        //}
        int n = marginalization_info->n;
        int m = marginalization_info->m;
        Eigen::VectorXd dx(n);
        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
            int size = marginalization_info->keep_block_size[i];
            int idx = marginalization_info->keep_block_idx[i] - m;
            Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
            Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
            if (size != 7)
                dx.segment(idx, size) = x - x0;
            else {
                dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
                dx.segment<3>(idx + 3) =
                        2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                                                Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
                if ((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                     Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() < 0) {
                    dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(
                            Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                            Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
                }
            }
        }
        Eigen::Map<Eigen::VectorXd>(residuals, n) =
                marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
        if (jacobians) {

            for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++) {
                if (jacobians[i]) {
                    int size = marginalization_info->keep_block_size[i], local_size = localSize(
                            size);
                    int idx = marginalization_info->keep_block_idx[i] - m;
                    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(
                            jacobians[i], n, size);
                    jacobian.setZero();
                    jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx,
                                                                                                          local_size);
                }
            }
        }
        return true;
    }
}