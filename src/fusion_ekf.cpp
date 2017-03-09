#include "fusion_ekf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF():
  ekf_(0, 0)
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  /**
  TODO:
    * Finish initializing the FusionEKF.
  */
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // Initialize the state ekf_.x with the first measurement.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double d = measurement_pack.raw_measurements_[0];
      double r = measurement_pack.raw_measurements_[1];
      double v = measurement_pack.raw_measurements_[2];

      double cos_r = cos(r);
      double sin_r = sin(r);

      ekf_.x << (d * cos_r), (d * sin_r), (v * cos_r), (v * sin_r);
    }
    else /* if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) */ {
      double p_x = measurement_pack.raw_measurements_[0];
      double p_y = measurement_pack.raw_measurements_[1];
      ekf_.x << p_x, p_y, 0, 0;
    }

    // Initialize timestamp.
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // Compute elapsed time in seconds.
  double timestamp = measurement_pack.timestamp_;
  double dt = (timestamp - previous_timestamp_) / 1000000.0;

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Predict new state.
  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x = " << ekf_.x << endl;
  cout << "P = " << ekf_.P << endl;

  // Update timestamp.
  previous_timestamp_ = timestamp;
}
