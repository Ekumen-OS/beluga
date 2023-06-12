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

#ifndef BELUGA_AMCL__AMCL_NODE_UTILS_HPP_
#define BELUGA_AMCL__AMCL_NODE_UTILS_HPP_

#include <utility>
#include <vector>

#include <beluga_amcl/ros_interfaces.hpp>

#include <sophus/se3.hpp>

namespace beluga_amcl::utils
{

/// Returns an Eigen format object with comma separators between coefficients.
/**
 * Useful for printing matrices and vectors in a single line.
 */
inline Eigen::IOFormat make_eigen_comma_format()
{
  return Eigen::IOFormat{
    Eigen::StreamPrecision,
    Eigen::DontAlignCols,
    ", ",  // coefficient separator
    ", ",  // row separator
  };
}

/// Generates a vector of points to update the measurement model.
/**
 * \param laser_scan Laser scan message to process.
 * \param laser_transform Transform from laser frame to robot frame.
 * \param max_beam_count Maximum number of evenly-spaced beams to be taken from the scan.
 * \param range_min Minimum range value. Beams with a range below this value will be ignored.
 * \param range_max Maximum range value. Beams with a range above this value will be ignored.
 * \return A vector of pairs of coordinates in the robot's reference frame.
 */
std::vector<std::pair<double, double>> make_points_from_laser_scan(
  const beluga_amcl::messages::LaserScan & laser_scan,
  const Sophus::SE3d & laser_transform,
  std::size_t max_beam_count,
  float range_min,
  float range_max);

}  // namespace beluga_amcl::utils

#endif  // BELUGA_AMCL__AMCL_NODE_UTILS_HPP_
