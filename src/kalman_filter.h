/*
 * Copyright (c) Helio Perroni Filho <xperroni@gmail.com>
 *
 * This file is part of KalmOn.
 *
 * KalmOn is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * KalmOn is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with KalmOn. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KALMON_KALMAN_FILTER_H
#define KALMON_KALMAN_FILTER_H

#include "sensors.h"
#include "state.h"

namespace kalmon {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief Kalman filter implementation.
 */
struct KalmanFilter {
  /** @brief Current process state. */
  State x;

  /** @brief State covariance matrix. */
  MatrixXd P;

  /**
   * @brief Default constructor.
   */
  KalmanFilter();

  /**
   * @brief Update the filter and return the current state.
   */
  State operator () (const Measurement z);

private:
  /** @brief Timestamp of last received measurement. */
  double t_;

  /** @brief State transition matrix. */
  MatrixXd F_;

  /** @brief Transposed state transition matrix. */
  MatrixXd Ft_;

  /** @brief Process covariance matrix. */
  MatrixXd Q_;

  /**
   * @brief Predict the state (and its covariance) of the process at the next step.
   *
   * @param dt Time difference between current and next steps.
   */
  void predict(double dt);

  /**
   * @brief Update the system state based on latest measurement.
   *
   * @param z Latest measurement.
   */
  void update(const Measurement z);
};

} // namespace kalmon

#endif
