#include "kalman_filter.h"

// identity matrix
static const MatrixXd I = MatrixXd::Identity(2, 2);

KalmanFilter::KalmanFilter(double noise_ax, double noise_ay):
  noise_ax_(noise_ax),
  noise_ay_(noise_ay),
  F_(MatrixXd(4, 4)),
  H_(MatrixXd(2, 4)),
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

  H_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;

  Ht_ = H_.transpose();

  Q_ <<
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}

void KalmanFilter::Predict(double dt) {
  // Update the state transition matrix
  // according to the new elapsed time.
  F_(0, 2) = Ft_(2, 0) = dt;
  F_(1, 3) = Ft_(3, 1) = dt;

  double dt2 = dt * dt;
  double dt3 = dt * dt2;
  double dt4 = dt * dt3;

  // Update the process noise covariance matrix
  // according to the new elapsed time.
  Q_(0, 0) = dt4 * noise_ax_ * 0.25;
  Q_(1, 1) = dt4 * noise_ay_ * 0.25;
  Q_(2, 2) = dt2 * noise_ax_;
  Q_(3, 3) = dt2 * noise_ay_;
  Q_(0, 2) = Q_(2, 0) = dt3 * noise_ax_ * 0.5;
  Q_(1, 3) = Q_(3, 1) = dt3 * noise_ay_ * 0.5;

  P = F_ * P * Ft_ + Q_;
}

MatrixXd KalmanFilter::Hj() const
{
  MatrixXd H(3, 4);
  H <<
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

  //recover state parameters
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  //check division by zero
  if (px == 0 && py == 0) {
      return H;
  }

  //compute the Jacobian matrix
  float px2 = px * px;
  float py2 = py * py;
  float d2 = px2 + py2;
  float d = sqrt(d2);
  float d3 = d2 * d;

  H(0, 0) = px / d;
  H(0, 1) = py / d;
  H(1, 0) = -py / d2;
  H(1, 1) = px / d2;
  H(2, 0) = py * (vx * py - vy * px) / d3;
  H(2, 1) = px * (vy * px - vx * py) / d3;
  H(2, 2) = px / d;
  H(2, 3) = py / d;

  return H;
}

void KalmanFilter::Update(const VectorXd &z) {
  MatrixXd H, Ht;
  if (z.rows() == 2) {
    H = H_;
    Ht = Ht_;
  }
  else /* if (m == 3) */ {
    H = Hj();
    Ht = H.transpose();
  }

  VectorXd y = z - H * x;
  MatrixXd S = H * P * Ht + R_;
  MatrixXd K = P * Ht * S.inverse();

  x = x + K * y;
  P = (I - K * H) * P;
}
