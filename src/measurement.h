#ifndef KALMON_MEASUREMENT_H
#define KALMON_MEASUREMENT_H

#include "state.h"

#include "Eigen/Dense"

#include <functional>
#include <iostream>
#include <memory>

namespace kalmon
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::istream;
using std::ostream;
using std::shared_ptr;

// Forward declaration.
struct Sensor;

/** @brief A function that computes the measurement covariance matrix R from given parameters. */
typedef std::function<MatrixXd()> MatrixR;

class Measurement: public VectorXd {
public:
  /** @brief Sensor used to collect the measurement. */
  shared_ptr<Sensor> sensor;

  /** @brief Timestamp for when this measurement was collected. */
  long timestamp;

  /** @brief Covariance matrix R for this measurement. */
  MatrixXd R;

  /**
   * @brief Default constructor.
   */
  Measurement();

  Measurement &operator = (const VectorXd &z);

  Measurement &operator = (VectorXd &&z);

  State x() const;

  /**
   * @brief Compute the measurement model matrix H for this measurement.
   */
  MatrixXd H(const VectorXd &x) const;

  friend istream &operator >> (istream &in, Measurement &z);

  friend ostream &operator << (ostream &out, const Measurement &z);
};

struct Sensor {
  /**
    * @brief Read a measurement from the given data stream.
    */
  virtual istream &read(istream &data, Measurement &z) const = 0;

  /**
    * @brief Write a measurement to the given data stream.
    */
  virtual ostream &write(const Measurement &z, ostream &out) const = 0;

  /**
    * @brief Convert given measurement to state.
    */
  virtual State toState(const Measurement &z) const = 0;

  /**
    * @brief Compute the measurement model matrix H for this sensor.
    */
  virtual MatrixXd H(const VectorXd &x) const = 0;
};

} // namespace kalmon

#endif
