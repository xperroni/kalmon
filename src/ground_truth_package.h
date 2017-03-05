#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd gt_values_;

  GroundTruthPackage()
  {
    gt_values_ = Eigen::VectorXd(4);
  }

  GroundTruthPackage(float x_gt, float y_gt, float vx_gt, float vy_gt)
  {
    gt_values_ = Eigen::VectorXd(4);
    gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
  }
};

#endif /* MEASUREMENT_PACKAGE_H_ */
