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

#include "sensors.h"

#include "settings.h"

using std::string;

namespace kalmon {

/**
 * @brief A measurement retrieved from the laser sensor.
 */
struct MeasurementLaser: Sensors::Measurement {
  /**
   * @brief Retrieve a new laser measurement from the given input stream.
   */
  MeasurementLaser(istream &data, const MatrixXd &H, const MatrixXd &R):
    H_(H),
    R_(R)
  {
    resize(2);
    Sensors::Measurement &z = *this;
    data >> z(0) >> z(1) >> z.timestamp;
  }

  virtual ostream &write(ostream &out) const {
      const Sensors::Measurement &z = *this;
      return out << z(0) << "\t" << z(1);
  }

  virtual State x() const {
    const Sensors::Measurement &z = *this;
    return State(z(0), z(1), 0, 0);
  }

  virtual MatrixXd H(const VectorXd &x) const {
    return H_;
  }

  virtual MatrixXd R() const {
    return R_;
  }

private:
  /** @brief Model matrix for this measurement. */
  const MatrixXd &H_;

  /** @brief Covariance matrix for this measurement. */
  const MatrixXd &R_;
};

/**
 * @brief A measurement retrieved from the radar sensor.
 */
struct MeasurementRadar: Sensors::Measurement {
  /**
   * @brief Retrieve a new radar measurement from the given input stream.
   */
  MeasurementRadar(istream &data, const MatrixXd &R):
    R_(R)
  {
    resize(3);
    Sensors::Measurement &z = *this;
    data >> z(0) >> z(1) >> z(2) >> z.timestamp;
  }

  virtual ostream &write(ostream &out) const {
    const Sensors::Measurement &z = *this;
    double r = z(0);
    double p = z(1);
    return out << r * ::cos(p) << "\t" << r * ::sin(p);
  }

  virtual State x() const {
    const Sensors::Measurement &z = *this;
    double d = z(0);
    double r = z(1);
    double v = z(2);

    double cos_r = ::cos(r);
    double sin_r = ::sin(r);

    return State(d * cos_r, d * sin_r, v * cos_r, v * sin_r);
  }

  virtual MatrixXd H(const VectorXd &x) const {
    MatrixXd H(3, 4);
    H <<
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    if (px == 0 && py == 0) {
        return H;
    }

    float px2 = px * px;
    float py2 = py * py;
    float d2 = px2 + py2;
    float d = ::sqrt(d2);
    float d3 = d2 * d;

    // Compute the Jacobian matrix of the state.
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

  virtual MatrixXd R() const {
    return R_;
  }

private:
  /** @brief Covariance matrix for this measurement. */
  const MatrixXd &R_;
};

Sensors::Sensors():
  laserH_(2, 4),
  laserR_(2, 2),
  radarR_(3, 3)
{
  laserH_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;

  double s2_px = getSettings().s2_px;
  double s2_py = getSettings().s2_py;
  laserR_ <<
    s2_px, 0,
    0, s2_py;

  double s2_d = getSettings().s2_d;
  double s2_r = getSettings().s2_r;
  double s2_v = getSettings().s2_v;

  radarR_ <<
    s2_d, 0, 0,
    0, s2_r, 0,
    0, 0, s2_v;
}

Sensors::Measurement *Sensors::operator () (istream &data) const {
    string sensor;
    data >> sensor;

    if (sensor == "L") {
      return new MeasurementLaser(data, laserH_, laserR_);
    }
    else /* if (sensor_type == "R") */ {
      return new MeasurementRadar(data, radarR_);
    }
}

istream &operator >> (istream &data, Measurement &z) {
  static Sensors sensors;
  z = sensors(data);
  return data;
}

ostream &operator << (ostream &data, Measurement &z) {
  return z->write(data);
}

} // namespace kalmon
