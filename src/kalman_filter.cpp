#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in) {
  F_ = F_in;
  x_ = x_in;
  P_ = P_in;
}

void KalmanFilter::Predict(const double deltaT, const double noise_ax, const double noise_ay) {
  // Source: From course material

  MatrixXd Q(4, 4);
  double dt_2 = deltaT * deltaT;
  double dt_3 = dt_2 * deltaT;
  double dt_4 = dt_3 * deltaT;

  Q << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
      0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
      dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
      0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

  // Updating dt on F_
  F_(0, 2) = deltaT;
  F_(1, 3) = deltaT;

  /** Predict the state */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q;
}

void KalmanFilter::Update(const VectorXd &z, Eigen::MatrixXd &H, Eigen::MatrixXd &R) {
  /** Update the state directly, no conversion needed */
  VectorXd y = z - (H * x_);
  UpdateState(y, H, R);
}

void KalmanFilter::UpdateEKF(const VectorXd &z, Eigen::MatrixXd &H, Eigen::MatrixXd &R) {
  /** Convert predicted state to polar coordinates */
  // Source: From course material
  VectorXd z_pred = VectorXd(3);
  z_pred = Tools::CartesianToPolar(x_(0), x_(1), x_(2), x_(3));  // encapsuled in a function
  VectorXd y = z - z_pred;
  y(1) = Tools::wrap_angle(y(1));  // previous line can create angles over pi and above -pi
  UpdateState(y, H, R);
}

void KalmanFilter::UpdateState(const Eigen::VectorXd &y, Eigen::MatrixXd &H, Eigen::MatrixXd &R) {
  // Source: From course material
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = (P_ * Ht) * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  // Update state + state covariance
  x_ = x_ + (K * y);
  P_ = (I - K * H) * P_;
}
