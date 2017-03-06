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

  // measurement matrix
  MatrixXd H_;

  // measurement matrix (transposed)
  MatrixXd Ht_;

  // measurement covariance matrix
  MatrixXd R_;

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
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
      MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);

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

  inline void setH(MatrixXd H)
  {
    H_ = H;
    Ht_ = H.transpose();
  }
};

#endif /* KALMAN_FILTER_H_ */
