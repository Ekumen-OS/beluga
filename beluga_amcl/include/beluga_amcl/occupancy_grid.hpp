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

#ifndef BELUGA_AMCL_OCCUPANCY_GRID_HPP
#define BELUGA_AMCL_OCCUPANCY_GRID_HPP

#include <tf2/utils.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <beluga/sensor/data/occupancy_grid.hpp>
#include <beluga_amcl/ros_interfaces.hpp>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace beluga_amcl {

class OccupancyGrid : public beluga::BaseOccupancyGrid2<OccupancyGrid> {
 public:
  struct ValueTraits {
    // https://wiki.ros.org/map_server#Value_Interpretation
    static constexpr std::int8_t kFreeValue = 0;
    static constexpr std::int8_t kUnknownValue = -1;
    static constexpr std::int8_t kOccupiedValue = 100;

    [[nodiscard]] static bool is_free(std::int8_t value) { return value == kFreeValue; }

    [[nodiscard]] static bool is_unknown(std::int8_t value) { return value == kUnknownValue; }

    [[nodiscard]] static bool is_occupied(std::int8_t value) { return value == kOccupiedValue; }
  };

  explicit OccupancyGrid(messages::OccupancyGridConstSharedPtr grid)
      : grid_(std::move(grid)), origin_(make_origin_transform(grid_->info.origin)) {}

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] std::size_t size() const { return grid_->data.size(); }

  [[nodiscard]] const auto& data() const { return grid_->data; }

  [[nodiscard]] std::size_t width() const { return grid_->info.width; }

  [[nodiscard]] std::size_t height() const { return grid_->info.height; }

  [[nodiscard]] double resolution() const { return grid_->info.resolution; }

  [[nodiscard]] static auto value_traits() { return ValueTraits{}; }

 private:
  messages::OccupancyGridConstSharedPtr grid_;
  Sophus::SE2d origin_;

  static Sophus::SE2d make_origin_transform(const messages::Pose& origin) {
    const auto rotation = Sophus::SO2d{tf2::getYaw(origin.orientation)};
    const auto translation = Eigen::Vector2d{origin.position.x, origin.position.y};
    return Sophus::SE2d{rotation, translation};
  }
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_OCCUPANCY_GRID_HPP
