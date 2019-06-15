#include "tools.h"
#include <iostream>
#include <math.h>
#include <iostream>
#include <cmath>


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Source: course material
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Source: course material
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}


VectorXd Tools::CartesianToPolar(const double px, const double py, const double vx, const double vy)
{
  // Source: course material
  Eigen::VectorXd polar_coordinates(3);
  const double range = sqrt(pow(px, 2) + pow(py, 2));
  const double bearing = atan2(py, px);
  const double range_rate = (px * vx + py * vy) / range;

  polar_coordinates << range, bearing, range_rate;
  return polar_coordinates;
}

double Tools::wrap_angle(double rad) {
  // Source: https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi/4635752
  // Copy the sign of the value in radians to the value of pi.
  double signed_pi = std::copysign(M_PI,rad);
  // Set the value of difference to the appropriate signed value between pi and -pi.
  rad = std::fmod(rad + signed_pi,(2 * M_PI)) - signed_pi;
  return rad;
}
