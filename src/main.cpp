#include "kalman_filter.h"
#include "sensors.h"
#include "state.h"

#include "Eigen/Dense"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <stdlib.h>

using namespace kalmon;

using namespace std;

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

  vector<Measurement> measurements;
  vector<State> ground_truth;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  for (;;) {
    Measurement z;
    State g;

    // Read measurement and ground truth from file stream.
    in_file >> z >> g;
    if (in_file.eof())
      break;

    measurements.push_back(z);

    // Read ground truth data to compare later.
    ground_truth.push_back(g);
  }

  // Create a Kalman filter instance.
  KalmanFilter filter;

  // State estimates.
  vector<State> estimates;

  //Call the EKF-based fusion
  for (size_t k = 0, n = measurements.size(); k < n; ++k) {
    Measurement z = measurements[k];
    State &g = ground_truth[k];

    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    State x = filter(z);

    cerr << "x:" << endl << filter.x << endl;
    cerr << "P:" << endl << filter.P << endl;


    // Output estimate, measurement and ground truth.
    out_file << x << '\t' << z << '\t' << g << endl;

    estimates.push_back(x);
  }

  // compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl << fixed << setprecision(2) << RMSE(estimates, ground_truth) << endl;

  // close files
  out_file.close();
  in_file.close();

  return 0;
}
