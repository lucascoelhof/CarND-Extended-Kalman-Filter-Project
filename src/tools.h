#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);


  static Eigen::VectorXd PolarToCartesian(double rho, double phi, double rhodot);
  static Eigen::VectorXd CartesianToPolar(const double px, const double py, const double vx, const double vy);
  static double wrap_angle(double angle);
};

#endif  // TOOLS_H_
