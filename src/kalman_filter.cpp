#include "kalman_filter.h"

#include "settings.h"

using namespace std;

namespace kalmon
{

// identity matrix
static const MatrixXd I = MatrixXd::Identity(4, 4);

KalmanFilter::KalmanFilter():
  F_(0, 0),
  Q_(MatrixXd(4, 4)),
  P(MatrixXd(4, 4))
{
  Q_ <<
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;
}

State KalmanFilter::operator () (const Measurement &z) {
  if (F_.rows() == 0) {
    F_.resize(4, 4);
    F_ <<
      1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

    Ft_ = F_.transpose();

    // Initialize state with first measurement.
    x = z->x();

    // Initialize covariance matrix.
    P = I * getSettings().s2_P0;

    // Initialize timestamp.
    t_ = z->timestamp;

    return x;
  }

  // Compute elapsed time in seconds.
  double t = z->timestamp;
  double dt = (t - t_) / 1000000.0;

  // Predict new state.
  predict(dt);

  // Update predictions.
  update(z);

  // Update timestamp.
  t_ = t;

  return x;
}

void KalmanFilter::predict(double dt) {
  // Update the state transition matrix
  // according to the new elapsed time.
  F_(0, 2) = Ft_(2, 0) = dt;
  F_(1, 3) = Ft_(3, 1) = dt;

  double dt2 = dt * dt;
  double dt3 = dt * dt2;
  double dt4 = dt * dt3;

  double s2_ax = getSettings().s2_ax;
  double s2_ay = getSettings().s2_ay;

  // Update the process noise covariance matrix
  // according to the new elapsed time.
  Q_(0, 0) = dt4 * s2_ax * 0.25;
  Q_(1, 1) = dt4 * s2_ay * 0.25;
  Q_(2, 2) = dt2 * s2_ax;
  Q_(3, 3) = dt2 * s2_ay;
  Q_(0, 2) = Q_(2, 0) = dt3 * s2_ax * 0.5;
  Q_(1, 3) = Q_(3, 1) = dt3 * s2_ay * 0.5;

  x = F_ * x;
  P = F_ * P * Ft_ + Q_;
}

void KalmanFilter::update(const Measurement z) {
  MatrixXd H = z->H(x);
  MatrixXd Ht = H.transpose();

  VectorXd y = *z - H * x;
  MatrixXd S = H * P * Ht + z->R();
  MatrixXd K = P * Ht * S.inverse();

  x += K * y;
  P = (I - K * H) * P;
}

}
