// Copyright 2022-2023 Ekumen, Inc.
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

#ifndef BELUGA_ROS_OCCUPANCY_GRID_HPP
#define BELUGA_ROS_OCCUPANCY_GRID_HPP

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <beluga/sensor/data/occupancy_grid.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * \file
 * \brief Implementation of `nav_msgs/OccupancyGrid` wrapper type.
 */

namespace beluga_ros {

/// Thin wrapper type for 2D `nav_msgs/OccupancyGrid` messages.
class OccupancyGrid : public beluga::BaseOccupancyGrid2<OccupancyGrid> {
 public:
  /// Traits for occupancy grid value interpretation.
  /**
   * Assumes a [standard ROS trinary interpretation](https://wiki.ros.org/map_server#Value_Interpretation).
   */
  struct ValueTraits {
    /// \brief Free value in the standard ROS trinary interpretation.
    static constexpr std::int8_t kFreeValue = 0;
    /// \brief Unknown value in the standard ROS trinary interpretation.
    static constexpr std::int8_t kUnknownValue = -1;
    /// \brief Occupied value in the standard ROS trinary interpretation.
    static constexpr std::int8_t kOccupiedValue = 100;

    /// Check if the given `value` corresponds to that of a free cell.
    [[nodiscard]] static bool is_free(std::int8_t value) { return value == kFreeValue; }

    /// Check if the given `value` corresponds to that of a cell of unknown occupancy.
    [[nodiscard]] static bool is_unknown(std::int8_t value) { return value == kUnknownValue; }

    /// Check if the given `value` corresponds to that of an occupied cell.
    [[nodiscard]] static bool is_occupied(std::int8_t value) { return value == kOccupiedValue; }
  };

  /// Constructor.
  ///
  /// \param grid Occupancy grid message.
  explicit OccupancyGrid(nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid)
      : grid_(std::move(grid)), origin_(make_origin_transform(grid_->info.origin)) {}

  /// Get the occupancy grid origin in the occupancy grid frame.
  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  /// Get the size of the occupancy grid (`width()` times `height()`).
  [[nodiscard]] std::size_t size() const { return grid_->data.size(); }

  /// Get a reference to the underlying data storeage (ie. a row-major array).
  [[nodiscard]] const auto& data() const { return grid_->data; }

  /// Get the width of the occupancy grid.
  [[nodiscard]] std::size_t width() const { return grid_->info.width; }

  /// Get the height of the occupancy grid.
  [[nodiscard]] std::size_t height() const { return grid_->info.height; }

  /// Get the resolution of the occupancy grid discretization, in meters.
  [[nodiscard]] double resolution() const { return grid_->info.resolution; }

  /// Get the traits for occupancy grid value interpretation.
  [[nodiscard]] static auto value_traits() { return ValueTraits{}; }

 private:
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;
  Sophus::SE2d origin_;

  static Sophus::SE2d make_origin_transform(const geometry_msgs::msg::Pose& origin) {
    const auto rotation = Sophus::SO2d{tf2::getYaw(origin.orientation)};
    const auto translation = Eigen::Vector2d{origin.position.x, origin.position.y};
    return Sophus::SE2d{rotation, translation};
  }
};

}  // namespace beluga_ros

#endif  // BELUGA_ROS_OCCUPANCY_GRID_HPP
