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

#ifndef SRC_INS_FACTOR_H
#define SRC_INS_FACTOR_H

//#include "../utility/utility.h"
//#include "../estimator/parameters.h"
//#include "integration_base.h"

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

struct INSRTError
{
    INSRTError(double t_x, double t_y, //double t_z
               double q_w, double q_x, double q_y, double q_z,
               double t_var, double q_var)
            :t_x(t_x), t_y(t_y),//, t_z(t_z),
             q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
             t_var(t_var), q_var(q_var){}

    template <typename T>
    bool operator()(const T* const w_P_i, const T* w_P_j, T* residuals) const
    {
        T t_w_ij[2];
        t_w_ij[0] = w_P_j[0] - w_P_i[0];
        t_w_ij[1] = w_P_j[1] - w_P_i[1];
//        t_w_ij[2] = w_P_j[2] - w_P_i[2];

        residuals[0] = (t_w_ij[0] - T(t_x)) / T(t_var);
        residuals[1] = (t_w_ij[1] - T(t_y)) / T(t_var);
//        residuals[2] = (t_w_ij[2] - T(t_z)) / T(t_var);

        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = - T(q_x);
        relative_q[2] = - T(q_y);
        relative_q[3] = - T(q_z);

        T q_w_j[4];
        q_w_j[0] = w_P_j[6];
        q_w_j[1] = w_P_j[3];
        q_w_j[2] = w_P_j[4];
        q_w_j[3] = w_P_j[5];

//        T relative_q_inv[4];
//        QuaternionInverse(relative_q, relative_q_inv);

        T error_q[4];
        ceres::QuaternionProduct(relative_q, q_w_j, error_q);

        residuals[2] = T(2) * error_q[1] / T(q_var);
        residuals[3] = T(2) * error_q[2] / T(q_var);
        residuals[4] = T(2) * error_q[3] / T(q_var);

        return true;
    }

    static ceres::CostFunction* Create(const double t_x, const double t_y, //const double t_z,
                                       const double q_w, const double q_x, const double q_y, const double q_z,
                                       const double t_var, const double q_var)
    {
        return (new ceres::AutoDiffCostFunction<
                INSRTError, 5, 7, 7>(
                new INSRTError(t_x, t_y, q_w, q_x, q_y, q_z, t_var, q_var)));
    }

    double t_x, t_y, t_z;
    double q_w, q_x, q_y, q_z;
    double t_var, q_var;

};


struct INSRError
{
    INSRError(double q_w, double q_x, double q_y, double q_z,
              double q_var)
            :q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
             q_var(q_var){}

    template <typename T>
    bool operator()(const T* w_P_j, T* residuals) const
    {
        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = - T(q_x);
        relative_q[2] = - T(q_y);
        relative_q[3] = - T(q_z);

        T q_w_j[4];
        q_w_j[0] = w_P_j[6];
        q_w_j[1] = w_P_j[3];
        q_w_j[2] = w_P_j[4];
        q_w_j[3] = w_P_j[5];

        T error_q[4];
        ceres::QuaternionProduct(relative_q, q_w_j, error_q);

        residuals[0] = T(2) * error_q[1] / T(q_var);
        residuals[1] = T(2) * error_q[2] / T(q_var);
        residuals[2] = T(2) * error_q[3] / T(q_var);

        return true;
    }

    static ceres::CostFunction* Create(const double q_w, const double q_x, const double q_y, const double q_z,
                                       const double q_var)
    {
        return (new ceres::AutoDiffCostFunction<
                INSRError, 3, 7>(
                new INSRError(q_w, q_x, q_y, q_z, q_var)));
    }

    double q_w, q_x, q_y, q_z;
    double q_var;

};






#endif //SRC_INS_FACTOR_H
