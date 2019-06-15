#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(const double deltaT, const double noise_ax, const double noise_ay);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z, Eigen::MatrixXd& H, Eigen::MatrixXd& R);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z, Eigen::MatrixXd& H, Eigen::MatrixXd& R);

  void UpdateState(const Eigen::VectorXd &y, Eigen::MatrixXd& H, Eigen::MatrixXd& R);

    // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  Eigen::MatrixXd F_;


};

#endif // KALMAN_FILTER_H_
