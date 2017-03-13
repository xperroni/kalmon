#include "measurement.h"

#include "settings.h"

using namespace std;

namespace kalmon
{

class Laser: public Sensor {
  MatrixXd H_;

public:
  Laser():
    H_(2, 4)
  {
    H_ <<
      1, 0, 0, 0,
      0, 1, 0, 0;
  }

  virtual istream &read(istream &data, Measurement &z) const {
    z.resize(2);
    return data >> z(0) >> z(1) >> z.timestamp;
  }

  virtual ostream &write(const Measurement &z, ostream &out) const
  {
      return out << z(0) << "\t" << z(1);
  }

  virtual State toState(const Measurement &z) const
  {
    State x;
    x << z(0), z(1), 0, 0;
    return x;
  }

  virtual MatrixXd H(const VectorXd &x) const {
    return H_;
  }
};

MatrixXd laserR() {
  double sx = getSettings().s_px;
  double sy = getSettings().s_py;

  MatrixXd R(2, 2);
  R <<
    sx, 0,
    0, sy;

  return R;
}

struct Radar: Sensor {
  virtual istream &read(istream &data, Measurement &z) const {
      z.resize(3);
      return data >> z(0) >> z(1) >> z(2) >> z.timestamp;
  }

  virtual ostream &write(const Measurement &z, ostream &out) const
  {
      double r = z(0);
      double p = z(1);
      return out << r * cos(p) << "\t" << r * sin(p);
  }

  virtual State toState(const Measurement &z) const
  {
    double d = z(0);
    double r = z(1);
    double v = z(2);

    double cos_r = cos(r);
    double sin_r = sin(r);

    return State(d * cos_r, d * sin_r, v * cos_r, v * sin_r);
  }

  virtual MatrixXd H(const VectorXd &x) const {
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
};

MatrixXd radarR() {
  double sd = getSettings().s_d;
  double sr = getSettings().s_r;
  double sv = getSettings().s_v;

  MatrixXd R(3, 3);
  R <<
    sd, 0, 0,
    0, sr, 0,
    0, 0, sv;

  return R;
}

static shared_ptr<Sensor> LASER(new Laser());

static shared_ptr<Sensor> RADAR(new Radar());

Measurement::Measurement():
  VectorXd(),
  timestamp(-1)
{
  // Nothing to do.
}

Measurement &Measurement::operator = (const VectorXd &z) {
  ((VectorXd) *this) = z;
  return *this;
}

Measurement &Measurement::operator = (VectorXd &&z) {
  ((VectorXd) *this) = z;
  return *this;
}

State Measurement::x() const {
  return sensor->toState(*this);
}

MatrixXd Measurement::H(const VectorXd &x) const {
  return sensor->H(x);
}

istream &operator >> (istream &in, Measurement &z) {
    string sensor_type;
    in >> sensor_type;

    if (sensor_type == "L") {
      z.sensor = LASER;
      z.R = laserR();
    }
    else /* if (sensor_type == "R") */ {
      z.sensor = RADAR;
      z.R = radarR();
    }

    return z.sensor->read(in, z);
}

ostream &operator << (ostream &out, const Measurement &z) {
  return z.sensor->write(z, out);
}

} // namespace kalmon
