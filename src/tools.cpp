#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  // Initialize RMSE error.
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check input validity.
  // 1) Estimation vector size not zero.
  // 2) Estimation vector and ground truth size should be the same.
	if (estimations.size() == 0) {
		cerr << "CalculateRMSE() - Error: Estimation is empty\n";
		return rmse;
	}
  if (estimations.size() != ground_truth.size()) {
    cerr << "CalculateRMSE() - Error: Invalid estimation or ground truth data\n";
    return rmse;
  }

  // Accumulate squared residuals.
  for (unsigned int i = 0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Mean calculation.
  rmse = rmse / estimations.size();

  // Square root calculation.
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  // Initialize Jacobian matrix.
  MatrixXd Hj(3, 4);
  Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  // Recover state parameters.
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Pre-compute terms to avoid repetitions.
  float c1 = px * px + py * py;

  // Check division by zero
	// 1) Return initialized Jacobian matrix
	// or
	// 2) Pick small value for px * py 
  if (fabs(c1) < 0.0001) {

		// (1)
		// std::cerr << "CalculateJacobian() - Error: Division by zero\n";
		//return Hj;

		// (2)
		c1 = 0.0001;
  }

  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  // Compute Jacobian matrix.
  Hj << (px / c2), (py / c2), 0, 0,
        -(py / c1), (px / c1), 0, 0,
        py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;
}
