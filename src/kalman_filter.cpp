#include "kalman_filter.h"

// identity matrix
static const MatrixXd I = MatrixXd::Identity(2, 2);

KalmanFilter::KalmanFilter(double noise_ax, double noise_ay):
  noise_ax_(noise_ax),
  noise_ay_(noise_ay),
  F_(MatrixXd(4, 4)),
  Q_(MatrixXd(4, 4)),
  R_(MatrixXd(2, 2)),
  x(VectorXd(4)),
  P(MatrixXd(4, 4))
{
  F_ <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;

  Ft_ = F_.transpose();

  Q_ <<
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x = x_in;
  P = P_in;
  R_ = R_in;
  Q_ = Q_in;
  setF(F_in);
  setH(H_in);
}

void KalmanFilter::Predict(double dt) {
  // Update the state transition matrix
  // according to the new elapsed time.
  F_(0, 2) = Ft_(2, 0) = dt;
  F_(1, 3) = Ft_(3, 1) = dt;

  double dt_2 = dt * dt;
  double dt_3 = dt * dt_2;
  double dt_4 = dt * dt_3;

  // Update the process noise covariance matrix
  // according to the new elapsed time.
  Q_(0, 0) = dt_4 * noise_ax_ * 0.25;
  Q_(1, 1) = dt_4 * noise_ay_ * 0.25;
  Q_(2, 2) = dt_2 * noise_ax_;
  Q_(3, 3) = dt_2 * noise_ay_;
  Q_(0, 2) = Q_(2, 0) = dt_3 * noise_ax_ * 0.5;
  Q_(1, 3) = Q_(3, 1) = dt_3 * noise_ay_ * 0.5;

  P = F_ * P * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x;
  MatrixXd S = H_ * P * Ht_ + R_;
  MatrixXd K = P * Ht_ * S.inverse();

  x = x + K * y;
  P = (I - K * H_) * P;
}
