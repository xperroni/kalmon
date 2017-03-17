#ifndef KALMON_SENSORS_H
#define KALMON_SENSORS_H

#include "state.h"

#include <iostream>
#include <memory>

namespace kalmon
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::istream;
using std::ostream;
using std::shared_ptr;

/**
 * @brief The sensor array used to retrieve location measurements.
 */
class Sensors {
  /** @brief Laser measurement model matrix. */
  MatrixXd laserH_;

  /** @brief Laser measurement covariance matrix. */
  MatrixXd laserR_;

  /** @brief Radar measurement covariance matrix. */
  MatrixXd radarR_;

public:
  /**
   * @brief A measurement retrieved from one of the sensors in the array.
   */
  struct Measurement: VectorXd {
    /** @brief Time of measurement retrieval. */
    double timestamp;

    /**
     * @brief Write this measurement to the given output stream.
     */
    virtual ostream &write(ostream &out) const = 0;

    /**
     * @brief Convert this measurement into an state estimate.
     */
    virtual State x() const = 0;

    /**
     * @brief Compute the model matrix H for this measurement.
     */
    virtual MatrixXd H(const VectorXd &x) const = 0;

    /**
     * @brief Compute the covariance matrix R for this measurement.
     */
    virtual MatrixXd R() const = 0;
  };

  /**
   * @brief Default constructor.
   */
  Sensors();

  /**
   * @brief Return a new sensor measurement from the given input stream.
   */
  Measurement *operator () (istream &data) const;
};

/**
 * @brief Reference-counting wrapper for sensor measurement objects.
 */
struct Measurement: shared_ptr<Sensors::Measurement> {
  /**
   * @brief Default constructor.
   */
  Measurement() {
    // Nothing to do.
  }

  /**
   * @brief Create a new wrapper for a sensor measurement object.
   */
  Measurement(Sensors::Measurement *z) {
    reset(z);
  }
};

/**
 * @brief Read a sensor measurement from the given input stream.
 */
istream &operator >> (istream &data, Measurement &z);

/**
 * @brief Write a sensor measurement to the given output stream.
 */
ostream &operator << (ostream &data, Measurement &z);


} // namespace kalmon

#endif
