#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {

  // Predicted state estimate
  x_ = F_ * x_;
  // Predicted estimate covariance
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::UpdateKFStep(const VectorXd &y) {

  MatrixXd H_t = H_.transpose();
  // Pre-fit residual covariance
  MatrixXd S = H_ * P_ * H_t + R_;
  // Optimal Kalman gain
  MatrixXd K = P_ * H_t * S.inverse();

  // Updated state estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // Updated estimate covariance
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {

  // Measurement pre-fit residual
  VectorXd y = z - H_ * x_;
  // Kalman update with pre-fit residual
  UpdateKFStep(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // Map predicted state to measurement space
  float x_x = x_(0);
  float x_y = x_(1);
  float x_vx = x_(2);
  float x_vy = x_(3);

  float rho = sqrt(x_x * x_x + x_y * x_y);
  float phi = atan2(x_y, x_x);
  float rho_dot = (x_x * x_vx + x_y * x_vy) / rho;

  VectorXd hx = VectorXd(3);
  hx << rho, phi, rho_dot;

  // Measurement pre-fit residual
  VectorXd y = z - hx;

	// Normalize angles
	while (y(1) < -M_PI) {
		y(1) += 2 * M_PI;
	}
	while (y(1) > M_PI) {
		y(1) -= 2 * M_PI;
	}

  // Kalman update with pre-fit residual
  UpdateKFStep(y);
}
