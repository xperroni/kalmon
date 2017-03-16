#include "state.h"

namespace kalmon
{

State::State():
  VectorXd(4)
{
  *this << 0, 0, 0, 0;
}

State::State(double px, double py, double vx, double vy):
  VectorXd(4)
{
  VectorXd &x = *this;
  x << px, py, vx, vy;
}

istream &operator >> (istream &in, State &x) {
  return in >> x(0) >> x(1) >> x(2) >> x(3);
}

ostream &operator << (ostream &out, const State &x) {
  return out << x(0) << "\t" << x(1) << "\t" << x(2) << "\t" << x(3);
}

State RMSE(const vector<State> &estimates, const vector<State> &ground_truth) {
  int n = n = estimates.size();
  State rmse;
  for(int i = 0; i < n; ++i) {
    const State &a = estimates[i];
    const State &b = ground_truth[i];

    State c = a - b;
    State d2 = c.array() * c.array();
    rmse += d2;
  }

  //calculate the mean
  rmse /= n;

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

} // namespace kalmon
