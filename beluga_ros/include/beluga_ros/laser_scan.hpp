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

#ifndef BELUGA_ROS_LASER_SCAN_HPP
#define BELUGA_ROS_LASER_SCAN_HPP

#include <range/v3/view/iota.hpp>

#include <beluga/sensor/data/laser_scan.hpp>
#include <beluga/views/take_evenly.hpp>
#include <beluga_ros/messages.hpp>

#include <sophus/se3.hpp>

/**
 * \file
 * \brief Implementation of `sensor_msgs/LaserScan` wrapper type.
 */

namespace beluga_ros {

/// Thin wrapper type for 2D `sensor_msgs/LaserScan` messages.
class LaserScan : public beluga::BaseLaserScan<LaserScan> {
 public:
  /// Range type.
  using Scalar = double;

  /// Constructor.
  ///
  /// \param scan Laser scan message.
  /// \param origin Laser scan frame origin in the filter frame.
  /// Note it is a transform in 3D because the frame lidars typically
  /// report data in is in general not coplanar with the plane on which
  /// 2D localization operates.
  /// \param max_beams Maximum number of beams to consider.
  /// \param min_range Minimum allowed range value (in meters).
  /// \param max_range Maximum allowed range value (in meters).
  explicit LaserScan(
      beluga_ros::msg::LaserScanConstSharedPtr scan,
      Sophus::SE3d origin = Sophus::SE3d(),
      std::size_t max_beams = std::numeric_limits<std::size_t>::max(),
      Scalar min_range = std::numeric_limits<Scalar>::min(),
      Scalar max_range = std::numeric_limits<Scalar>::max())
      : scan_(std::move(scan)),
        origin_(std::move(origin)),
        max_beams_(max_beams),
        min_range_(std::max(static_cast<Scalar>(scan_->range_min), min_range)),
        max_range_(std::min(static_cast<Scalar>(scan_->range_max), max_range)) {
    assert(scan_ != nullptr);
  }

  /// Get the laser scan frame origin in the filter frame.
  [[nodiscard]] const auto& origin() const { return origin_; }

  /// Get laser scan measurement angles as a range.
  [[nodiscard]] auto angles() const {
    return ranges::views::iota(0, static_cast<int>(scan_->ranges.size())) | beluga::views::take_evenly(max_beams_) |
           ranges::views::transform([this](int i) {
             return static_cast<Scalar>(scan_->angle_min + static_cast<float>(i) * scan_->angle_increment);
           });
  }

  /// Get laser scan range measurements as a range.
  [[nodiscard]] auto ranges() const {
    return scan_->ranges | beluga::views::take_evenly(max_beams_) |
           ranges::views::transform([](auto value) { return static_cast<Scalar>(value); });
  }

  /// Get the minimum range measurement.
  [[nodiscard]] auto min_range() const { return min_range_; }

  /// Get the maximum range measurement.
  [[nodiscard]] auto max_range() const { return max_range_; }

 private:
  beluga_ros::msg::LaserScanConstSharedPtr scan_;
  Sophus::SE3d origin_;
  std::size_t max_beams_;
  Scalar min_range_;
  Scalar max_range_;
};

}  // namespace beluga_ros

#endif  // BELUGA_ROS_LASER_SCAN_HPP
