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

#include "kalman_filter.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace kalmon;

using namespace std;

/**
 * @brief Make sure user has provided input and output files.
 */
void check_arguments(int argc, char* argv[]) {
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

/**
 * @brief Try to open given file, quit program if unsuccessful.
 */
template<class T> void open(T &file, const char *path) {
  file.open(path);
  if (!file.is_open()) {
    cerr << "Cannot open file \"" << path << '"' << endl;
    exit(EXIT_FAILURE);
  }
}

/**
 * @brief Run program.
 */
int main(int argc, char* argv[]) {
  check_arguments(argc, argv);

  ifstream data;
  open(data, argv[1]);

  ofstream out;
  open(out, argv[2]);

  vector<Measurement> measurements;
  vector<State> ground_truth;

  // Read input measurements and ground truth.
  for (;;) {
    Measurement z;
    State g;

    data >> z >> g;
    if (data.eof())
      break;

    measurements.push_back(z);
    ground_truth.push_back(g);
  }

  // Create a Kalman filter instance.
  KalmanFilter filter;

  // State estimates.
  vector<State> estimates;

  // Produce state estimates for loaded measurements and
  // record them along ground truth values.
  for (size_t k = 0, n = measurements.size(); k < n; ++k) {
    Measurement z = measurements[k];
    State x = filter(z);

    cerr << "x:" << endl << filter.x << endl;
    cerr << "P:" << endl << filter.P << endl;

    out << x << '\t' << z << '\t' << ground_truth[k] << endl;

    estimates.push_back(x);
  }

  // Compute overall estimate accuracy using Root Mean Squared Error (RMSE).
  cerr << "Accuracy - RMSE:" << endl << fixed << setprecision(2) << RMSE(estimates, ground_truth) << endl;

  out.close();
  data.close();

  return 0;
}
