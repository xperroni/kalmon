#ifndef KALMON_KALMAN_FILTER_H
#define KALMON_KALMAN_FILTER_H

#include "measurement.h"
#include "state.h"

namespace kalmon
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
  // Previous timestamp.
  double t_;

  // state transistion matrix
  MatrixXd F_;

  // state transition matrix (transposed)
  MatrixXd Ft_;

  // process covariance matrix
  MatrixXd Q_;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  void predict(double dt);

  /**
   * Updates the state and
   * @param z The measurement at k+1
   */
  void update(const Measurement &z);

public:
  // state vector
  State x;

  // state covariance matrix
  MatrixXd P;

  /**
   * @brief Create a new Kalman Filter.
   */
  KalmanFilter();

  /**
   * @brief Update the filter and return the current state.
   */
  State operator () (const Measurement &z);
};

} // namespace kalmon

#endif
