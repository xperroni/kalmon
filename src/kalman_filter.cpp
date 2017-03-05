#include "kalman_filter.h"

// identity matrix
static const MatrixXd I = MatrixXd::Identity(2, 2);

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x = x_in;
  P = P_in;
  R_ = R_in;
  Q_ = Q_in;
  setF(F_in);
  setH(H_in);
}

void KalmanFilter::Predict() {
  x = F_ * x;
  P = F_ * P * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x;
  MatrixXd S = H_ * P * Ht_ + R_;
  MatrixXd K = P * Ht_ * S.inverse();

  x = x + K * y;
  P = (I - K * H_) * P;
}
