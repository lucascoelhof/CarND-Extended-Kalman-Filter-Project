#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    VectorXd x(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];

      double xp = rho * cos(phi);
      double yp = rho * sin(phi);
      x << xp, yp, 0, 0;

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x << measurement_pack.raw_measurements_[0], // px
          measurement_pack.raw_measurements_[1], // py
          0,                                     // vx = unknown
          0;                                     // vy = unknown
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    //state covariance matrix P
    Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
    P << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    // Initializing F
    Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
    F << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    ekf_.Init(x, P, F);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  double deltaT = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.Predict(deltaT, 9, 9);  // using noise = 9  as suggested

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    MatrixXd Hj = Tools::CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, Hj, R_radar_);

  } else { // sensor_type_ == MeasurementPackage::LASER
    /** Measurement from Lidar can be used directly to update the filter. */
    ekf_.Update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
  }


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
