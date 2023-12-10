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

#include <beluga_amcl/amcl_node_utils.hpp>

namespace beluga_amcl::utils {

std::vector<std::pair<double, double>> make_points_from_laser_scan(
    const beluga_ros::msg::LaserScan& laser_scan,
    const Sophus::SE3d& laser_transform,
    std::size_t max_beam_count,
    float range_min,
    float range_max) {
  const std::size_t beam_count = laser_scan.ranges.size();
  range_min = std::max(laser_scan.range_min, range_min);
  range_max = std::min(laser_scan.range_max, range_max);

  auto points = std::vector<std::pair<double, double>>{};

  if (max_beam_count <= 1 || beam_count <= 1) {
    return points;  // Return empty vector
  }

  const std::size_t step = std::max(1UL, (beam_count - 1) / (max_beam_count - 1));
  points.reserve(std::min(beam_count, max_beam_count));

  for (std::size_t index = 0; index < beam_count; index += step) {
    const float range = laser_scan.ranges[index];
    if (std::isnan(range) || range <= range_min || range >= range_max) {
      continue;
    }
    // Store points in the robot's reference frame.
    // Assume that laser scanning is instantaneous and no compensation is
    // needed for robot speed vs. scan speed.
    const float angle = laser_scan.angle_min + static_cast<float>(index) * laser_scan.angle_increment;
    const auto point = laser_transform * Eigen::Vector3d{range * std::cos(angle), range * std::sin(angle), 0.0};
    points.emplace_back(point.x(), point.y());
  }
  return points;
}

}  // namespace beluga_amcl::utils
