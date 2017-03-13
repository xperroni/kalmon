#include "tools.h"

#include <iostream>

namespace kalmon
{

State Tools::CalculateRMSE(const vector<State> &estimates, const vector<State> &ground_truth) {
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

}
