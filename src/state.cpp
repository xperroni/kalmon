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

#include "state.h"

namespace kalmon {

State::State():
  VectorXd(4)
{
  *this << 0, 0, 0, 0;
}

State::State(double px, double py, double vx, double vy):
  VectorXd(4)
{
  *this << px, py, vx, vy;
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

  rmse /= n;

  rmse = rmse.array().sqrt();

  return rmse;
}

} // namespace kalmon
