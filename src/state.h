#ifndef KALMON_STATE_H
#define KALMON_STATE_H

#include "Eigen/Dense"

#include <iostream>

namespace kalmon
{

using Eigen::EigenBase;
using Eigen::VectorXd;

using std::istream;
using std::ostream;

class State: public VectorXd {
public:
  State();

  State(double px, double py, double vx, double vy);

  template<class T> State(const EigenBase<T> &x);
};

template<class T> State::State(const EigenBase<T> &x):
  VectorXd(4)
{
  VectorXd &values = *this;
  values = x;
}

istream &operator >> (istream &in, State &x);

ostream &operator << (ostream &out, const State &x);

} // namespace kalmon

#endif
