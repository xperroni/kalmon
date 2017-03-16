#ifndef KALMON_STATE_H
#define KALMON_STATE_H

#include "Eigen/Dense"

#include <iostream>
#include <vector>

namespace kalmon
{

using Eigen::EigenBase;
using Eigen::VectorXd;

using std::istream;
using std::ostream;
using std::vector;

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

State RMSE(const vector<State> &estimates, const vector<State> &ground_truth);

} // namespace kalmon

#endif
