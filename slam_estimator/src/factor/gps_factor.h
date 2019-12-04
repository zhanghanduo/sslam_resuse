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

#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
}


struct TError
{
	TError(double t_x, double t_y, double t_z, double var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);
		residuals[2] = (tj[2] - T(t_z)) / T(var);
		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var)
	{
	  return (new ceres::AutoDiffCostFunction<
	          TError, 3, 3>(
	          	new TError(t_x, t_y, t_z, var)));
	}

	double t_x, t_y, t_z, var;

};

struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z,
					double q_w, double q_x, double q_y, double q_z,
					double t_var1, double t_var2, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z),
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
                   t_var1(t_var1), t_var2(t_var2), q_var(q_var){}

    template<typename T>
    bool operator()(const T *const w_P_i, const T *w_P_j, T *residuals) const {
        T t_w_ij[2];
        t_w_ij[0] = w_P_j[0] - w_P_i[0];
        t_w_ij[1] = w_P_j[1] - w_P_i[1];
        t_w_ij[2] = w_P_j[2] - w_P_i[2];

        residuals[0] = (t_w_ij[0] - T(t_x)) / T(t_var1);
        residuals[1] = (t_w_ij[1] - T(t_y)) / T(t_var2);
        residuals[2] = (t_w_ij[2] - T(t_z)) / T(t_var2);

        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = -T(q_x);
        relative_q[2] = -T(q_y);
        relative_q[3] = -T(q_z);

        T q_w_j[4];
        q_w_j[0] = w_P_j[6];
        q_w_j[1] = w_P_j[3];
        q_w_j[2] = w_P_j[4];
        q_w_j[3] = w_P_j[5];

//        T relative_q_inv[4];
//        QuaternionInverse(relative_q, relative_q_inv);

        T error_q[4];
        ceres::QuaternionProduct(relative_q, q_w_j, error_q);

        residuals[3] = T(2) * error_q[1] / T(q_var);
        residuals[4] = T(2) * error_q[2] / T(q_var);
        residuals[5] = T(2) * error_q[3] / T(q_var);

        return true;
    }

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var1, const double t_var2, const double q_var)
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 7, 7>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var1, t_var2, q_var)));
	}

	double t_x, t_y, t_z;
	double q_w, q_x, q_y, q_z;
	double t_var1, t_var2, q_var;
};