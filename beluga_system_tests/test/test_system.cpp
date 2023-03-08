// Copyright 2023 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <sophus/se2.hpp>

#include <beluga/algorithm/particle_filter.hpp>
#include <beluga_amcl/occupancy_grid.hpp>
#include <rosbag2_cpp/reader.hpp>

namespace {

struct OccupancyGrid {};

struct InitialPose {
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance;
};

struct TestDataPoint {
  Sophus::SE2d odom;
  std::vector<std::pair<double, double>> scan;
  std::vector<Sophus::SE2d> ground_truth;
};

struct TestData {
  OccupancyGrid map;
  InitialPose initial_pose;
  std::vector<TestDataPoint> data_points;
  double tol_distance;
  double tol_angle;
};

}  // namespace
