#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
  // Process noise variance across the X axis.
  double noise_ax_;

  // Process noise variance across the Y axis.
  double noise_ay_;

  // state transistion matrix
  MatrixXd F_;

  // state transition matrix (transposed)
  MatrixXd Ft_;

  // process covariance matrix
  MatrixXd Q_;

  // linear measurement matrix
  MatrixXd H_;

  // linear measurement matrix (transposed)
  MatrixXd Ht_;

  // measurement covariance matrix
  MatrixXd R_;

  // Compute the Jacobian matrix for radar measurement.
  MatrixXd Hj() const;

public:
  // state vector
  VectorXd x;

  // state covariance matrix
  MatrixXd P;

  /**
   * @brief Create a new Kalman Filter with given variance settings.
   */
  KalmanFilter(double noise_ax, double noise_ay);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  void Predict(double dt);

  /**
   * Updates the state and
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */
