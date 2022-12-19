// Copyright 2022 Ekumen, Inc.
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

#ifndef BELUGA_AMCL__OCCUPANCY_GRID_HPP_
#define BELUGA_AMCL__OCCUPANCY_GRID_HPP_

#include <tf2/utils.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace beluga_amcl
{

class OccupancyGrid
{
public:
  struct Traits
  {
    // https://wiki.ros.org/map_server#Value_Interpretation

    static bool is_free(std::int8_t value)
    {
      return value == 0;
    }

    static bool is_unknown(std::int8_t value)
    {
      return value == -1;
    }

    static bool is_occupied(std::int8_t value)
    {
      return value == 100;
    }
  };

  explicit OccupancyGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid)
  : grid_{std::move(grid)},
    origin_{make_origin_transform(grid_->info.origin)} {}

  std::size_t size() const
  {
    return grid_->data.size();
  }

  const auto & data() const
  {
    return grid_->data;
  }

  const Sophus::SE2d & origin() const
  {
    return origin_;
  }

  std::size_t index(double x, double y) const
  {
    const auto x_index = static_cast<std::size_t>(std::floor(x / resolution()));
    const auto y_index = static_cast<std::size_t>(std::floor(y / resolution()));
    if (x_index >= width() || y_index >= height()) {
      return size();  // If the point is outside the map, return an invalid index
    }
    return x_index + y_index * width();
  }

  std::size_t index(const Eigen::Vector2d & point) const
  {
    return index(point.x(), point.y());
  }

  Eigen::Vector2d point(std::size_t index) const
  {
    return Eigen::Vector2d{
      (static_cast<double>(index % width()) + 0.5) * resolution(),
      (static_cast<double>(index / width()) + 0.5) * resolution()};
  }

  auto neighbors(std::size_t index) const
  {
    auto result = std::vector<std::size_t>{};
    const std::size_t row = index / width();
    const std::size_t col = index % width();
    if (row < (height() - 1)) {
      result.push_back(index + width());
    }
    if (row > 0) {
      result.push_back(index - width());
    }
    if (col < (width() - 1)) {
      result.push_back(index + 1);
    }
    if (col > 0) {
      result.push_back(index - 1);
    }
    return result;
  }

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  Sophus::SE2d origin_;

  std::size_t width() const
  {
    return grid_->info.width;
  }

  std::size_t height() const
  {
    return grid_->info.height;
  }

  double resolution() const
  {
    return grid_->info.resolution;
  }

  static Sophus::SE2d make_origin_transform(const geometry_msgs::msg::Pose & origin)
  {
    const auto rotation = Sophus::SO2d{tf2::getYaw(origin.orientation)};
    const auto translation = Eigen::Vector2d{origin.position.x, origin.position.y};
    return Sophus::SE2d{rotation, translation};
  }
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__OCCUPANCY_GRID_HPP_
