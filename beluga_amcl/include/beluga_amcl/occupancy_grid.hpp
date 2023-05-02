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
    : grid_{std::move(grid)}, origin_{make_origin_transform(grid_->info.origin)}
  {
  }

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

  Eigen::Vector2i cell(double x, double y) const
  {
    const auto xi = static_cast<std::size_t>(
        std::max(std::floor(x / resolution()), 0.));
    const auto yi = static_cast<std::size_t>(
        std::max(std::floor(y / resolution()), 0.));
    return Eigen::Vector2i{
      static_cast<int>(std::min(xi, width() - 1)),
      static_cast<int>(std::min(yi, height() - 1))};
  }

  Eigen::Vector2i cell(const Eigen::Vector2d & point) const
  {
    return cell(point.x(), point.y());
  }

  std::size_t index(int xi, int yi) const
  {
    if (xi < 0 || static_cast<std::size_t>(xi) >= width()) {
      return size();
    }
    if (yi < 0 || static_cast<std::size_t>(yi) >= height()) {
      return size();
    }
    return xi + yi * width();
  }

  std::size_t index(const Eigen::Vector2i & cell) const
  {
    return index(cell.x(), cell.y());
  }

  std::size_t index(double x, double y) const
  {
    const auto xi = static_cast<int>(std::floor(x / resolution()));
    const auto yi = static_cast<int>(std::floor(y / resolution()));
    return index(xi, yi);
  }

  std::size_t index(const Eigen::Vector2d & point) const
  {
    return index(point.x(), point.y());
  }

  Eigen::Vector2d point(int xi, int yi) const
  {
    return Eigen::Vector2d{
      (static_cast<double>(xi) + 0.5) * resolution(),
      (static_cast<double>(yi) + 0.5) * resolution()};
  }

  Eigen::Vector2d point(const Eigen::Vector2i & cell) const
  {
    return point(cell.x(), cell.y());
  }

  Eigen::Vector2d point(std::size_t index) const
  {
    return point(static_cast<int>(index % width()),
                 static_cast<int>(index / width()));
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

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  Sophus::SE2d origin_;

  static Sophus::SE2d make_origin_transform(const geometry_msgs::msg::Pose & origin)
  {
    const auto rotation = Sophus::SO2d{tf2::getYaw(origin.orientation)};
    const auto translation = Eigen::Vector2d{origin.position.x, origin.position.y};
    return Sophus::SE2d{rotation, translation};
  }
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__OCCUPANCY_GRID_HPP_
