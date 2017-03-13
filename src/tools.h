#ifndef TOOLS_H_
#define TOOLS_H_

#include "state.h"

#include "Eigen/Dense"

#include <vector>

namespace kalmon
{

using namespace std;

class Tools {
public:
  /**
  * A helper method to calculate RMSE.
  */
  State CalculateRMSE(const vector<State> &estimations, const vector<State> &ground_truth);
};

} // namespace kalmon

#endif /* TOOLS_H_ */
