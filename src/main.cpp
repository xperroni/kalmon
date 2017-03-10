#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "fusion_ekf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void check_arguments(int argc, char* argv[]) {
  // Make sure user has provided input and output files.
  if (argc == 2) {
    cerr << "Please include an output file." << endl;
  }
  else if (argc > 3) {
    cerr << "Too many arguments." << endl;
  }

  if (argc != 3) {
    cerr << "Usage instructions: " << argv[0] << " path/to/input.txt output.txt" << endl;
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

template<class T> void open(T &file, const char *path) {
  file.open(path);
  if (!file.is_open()) {
    cerr << "Cannot open file \"" << path << '"' << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {
  // Confirm arguments are correct, quits otherwise.
  check_arguments(argc, argv);

  ifstream in_file;
  open(in_file, argv[1]);

  ofstream out_file;
  open(out_file, argv[2]);

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (!in_file.eof()) {
    MeasurementPackage meas_package;
    long timestamp;

    // reads first element from the current line
    string sensor_type;
    in_file >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      in_file >> meas_package.raw_measurements_(0);
      in_file >> meas_package.raw_measurements_(1);
      in_file >> meas_package.timestamp_;

      measurement_pack_list.push_back(meas_package);
    }
    else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      in_file >> ro;
      in_file >> theta;
      in_file >> ro_dot;
      meas_package.raw_measurements_ << ro, theta, ro_dot;
      in_file >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    in_file >> x_gt;
    in_file >> y_gt;
    in_file >> vx_gt;
    in_file >> vy_gt;
    gt_pack_list.emplace_back(x_gt, y_gt, vx_gt, vy_gt);
  }

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    out_file << fusionEKF.ekf_.x(0) << "\t";
    out_file << fusionEKF.ekf_.x(1) << "\t";
    out_file << fusionEKF.ekf_.x(2) << "\t";
    out_file << fusionEKF.ekf_.x(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation
      out_file << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file << ro * cos(phi) << "\t"; // p1_meas
      out_file << ro * sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    out_file << gt_pack_list[k].gt_values_(0) << "\t";
    out_file << gt_pack_list[k].gt_values_(1) << "\t";
    out_file << gt_pack_list[k].gt_values_(2) << "\t";
    out_file << gt_pack_list[k].gt_values_(3) << "\n";

    estimations.push_back(fusionEKF.ekf_.x);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file.is_open()) {
    out_file.close();
  }

  if (in_file.is_open()) {
    in_file.close();
  }

  return 0;
}
