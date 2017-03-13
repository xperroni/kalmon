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

istream &operator >> (istream &in, State &x)
{
  return in >> x(0) >> x(1) >> x(2) >> x(3);
}

ostream &operator << (ostream &out, const State &x)
{
  return out << x(0) << "\t" << x(1) << "\t" << x(2) << "\t" << x(3);
}

} // namespace kalmon
