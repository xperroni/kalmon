#include <iostream>
#include "tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);

  rmse << 0, 0, 0, 0;
  int n = estimations.size();
  for(int i = 0; i < n; ++i) {
    const VectorXd &a = estimations[i];
    const VectorXd &b = ground_truth[i];

    VectorXd c = a - b;
    VectorXd d2 = c.array() * c.array();
    rmse += d2;
  }

  //calculate the mean
  rmse /= n;

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}
