/**
* Implementation of KalmanFilter class. Modified by phamngtuananh.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>
#include <utility>
#include "../../../slam_estimator/src/utility/utility.h"
#include "kalman.h"

using namespace std;

PoseKalmanFilter::PoseKalmanFilter(double dt,
                                   const Eigen::MatrixXd& Q,
                                   const Eigen::MatrixXd& R,
                                   const Eigen::MatrixXd& P)
  : m_(R.rows()), n_(Q.rows()), Q_(Q), R_(R), P_(P), P0_(P),
    I_(18, 18), dt_(dt), initialized_(false),
    x_hat_(n_), x_hat_new_(n_)
{
  if(Q.rows() != 18 || Q.cols() != 18)
    cout << "[KalmanFilter] WARN: <Q> has invalid dimension!" << endl;
  if(R.rows() != 6 || R.cols() != 6)
    cout << "[KalmanFilter] WARN: <R> has invalid dimension!" << endl;
  if(P.rows() != 18 || P.cols() != 18)
    cout << "[KalmanFilter] WARN: <P> has invalid dimension!" << endl;
  if(m_ != 6 || n_ != 18)
    cout << "[KalmanFilter] WARN: System dimensions are invalid!" << endl;
  A_ = createSystemDynamics(dt);
  H_ = Eigen::MatrixXd::Identity(m_, n_);
//  cout << "H " << endl << H_ << endl;
  I_ = Eigen::MatrixXd::Identity(n_, n_);
}

PoseKalmanFilter::PoseKalmanFilter(): m_(6), n_(18), initialized_(false) {}

///////////////////////////////////////////////////////////////////////////////
void PoseKalmanFilter::init()
{
  x_hat_.setZero();
  t0_ = 0;
  t_ = 0;
  initialized_ = true;
}

void PoseKalmanFilter::init(const double t0, const Eigen::VectorXd& x0)
{
  x_hat_ = x0;
  t0_ = t0;
  t_ = t0;
  initialized_ = true;
}

void PoseKalmanFilter::init(const double t0, const Eigen::Vector3d& z0, const Eigen::Matrix3d& r0)
{
	Eigen::VectorXd x0 = Eigen::VectorXd::Zero(18);
	x0.head(3) = z0;
	Eigen::Vector3d r_3 = Utility::R2ypr(r0);
	x0[3] = r_3.z();    // roll (around X axis)
	x0[4] = r_3.y();    // pitch (around Y axis)
	x0[5] = r_3.x();    // yaw (around Z axis)

	init(t0, x0);
}

 /**
 * Update the estimated state with measurement value.
 */
 void PoseKalmanFilter::update(const Eigen::VectorXd& z)
{
  if(!initialized_)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new_ = A_ * x_hat_;

  P_ = A_ * P_ * A_.transpose() + Q_;

  Eigen::MatrixXd K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
//	cout << "K: " << K.rows() << " x " << K.cols() << endl
//	     << "H_: " << H_.rows() << " x " << H_.cols() << endl
//	     << "R_: " << R_.rows() << " x " << R_.cols()<< endl;

  x_hat_new_ += K * (z - H_ * x_hat_new_);
//	 cout << "x_hat_new: " << x_hat_new_.rows() << " x " << x_hat_new_.cols() << endl
//	      << "z: " << z.rows() << " x " << z.cols() << endl
//	      << "x_hat: " << x_hat_.rows() << " x " << x_hat_.cols()<< endl;

  P_ = (I_ - K * H_) * P_;
  x_hat_ = x_hat_new_;
}

void PoseKalmanFilter::update(const double dt, const Eigen::VectorXd& z, const Eigen::MatrixXd& A)
{
  A_ = A;
  dt_ = dt;
  t_ += dt_;
  update(z);
}

void PoseKalmanFilter::update(const double t, const Eigen::Vector3d& z, const Eigen::Matrix3d& r)
{
	double dt = t - t_;
	t_ = t;
	A_ = createSystemDynamics(dt);
	Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(6);
	x_hat.head(3) = z;
	Eigen::Vector3d r_3 = Utility::R2ypr(r);
	x_hat[3] = r_3.z();    // roll (around X axis)
	x_hat[4] = r_3.y();    // pitch (around Y axis)
	x_hat[5] = r_3.x();    // yaw (around Z axis)
	update(x_hat);
}

///////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd PoseKalmanFilter::createSystemDynamics(double dt)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(18, 18);
  double dt2 = dt * dt / 2;
  A(0, 6)   = dt;
  A(1, 7)   = dt;
  A(2, 8)   = dt;
  A(3, 9)   = dt;
  A(4, 10)  = dt;
  A(5, 11)  = dt;
  A(6, 12)  = dt;
  A(7, 13)  = dt;
  A(8, 14)  = dt;
  A(9, 15)  = dt;
  A(10, 16) = dt;
  A(11, 17) = dt;
  A(0, 12) = dt2;
  A(1, 13) = dt2;
  A(2, 14) = dt2;
  A(3, 15) = dt2;
  A(4, 16) = dt2;
  A(5, 17) = dt2;
  return A;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd PoseKalmanFilter::getRawState() {return x_hat_;}

void PoseKalmanFilter::getPoseState(Eigen::Vector3d& z, Eigen::Matrix3d& r)
{
  z = x_hat_.head(3);
  r = Utility::ypr2R(Eigen::Vector3d(x_hat_[5], x_hat_[4], x_hat_[3]));
}


double PoseKalmanFilter::getTime() {return t_;}
