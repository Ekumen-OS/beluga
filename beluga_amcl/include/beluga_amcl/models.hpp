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

/*
 * Note: the contents of this file should probably be moved to the
 * core library or replaced with external libraries (state representation).
 */

#ifndef BELUGA_AMCL__MODELS_HPP_
#define BELUGA_AMCL__MODELS_HPP_

#include <beluga/algorithm/distance_map.h>
#include <beluga/spatial_hash.h>

#include <algorithm>
#include <cmath>
#include <queue>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <range/v3/view.hpp>

namespace beluga_amcl
{

// TODO(nahuel): Replace this short-term solution with a fully-featured
// vector state representation.
struct Pose
{
  double x;
  double y;
  double theta;

  explicit operator geometry_msgs::msg::Pose() const noexcept
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0;
    pose.orientation.w = std::cos(theta / 2.);
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = std::sin(theta / 2.);
    return pose;
  }
};

// TODO(nahuel): Replace this with a real motion model.
template<class Mixin>
class MockMotionModel : public Mixin
{
public:
  template<class ... Args>
  explicit MockMotionModel(Args && ... args)
  : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] auto apply_motion(const Pose & state) {return state;}
};

struct LikelihoodSensorModelParam
{
  double max_obstacle_distance;
  double max_laser_distance;
  double z_hit;
  double z_random;
  double sigma_hit;
};

// TODO(nahuel): Move this to the core package once it doesn't depend on
// ROS message types, or if we decide to add that dependecy to the core.
template<class Mixin>
class LikelihoodSensorModel : public Mixin
{
public:
  template<class ... Args>
  explicit LikelihoodSensorModel(
    const LikelihoodSensorModelParam & params,
    const nav_msgs::msg::OccupancyGrid & map, Args && ... rest)
  : Mixin(std::forward<Args>(rest)...),
    params_{params},
    map_metadata_{map.info},
    free_cells_{make_free_cells_list(map)},
    index_distribution_{0, free_cells_.size() - 1},
    likelihood_field_{make_likelihood_field(params, map)}
  {}

  [[nodiscard]] double importance_weight(const Pose &)
  {
    // TODO(nahuel): Implement likelihood estimation.
    return 1.;
  }

  template<class Generator>
  [[nodiscard]] auto generate_random_state(Generator & generator)
  {
    auto pose = Pose{};
    auto [x, y] = xy_from_index(map_metadata_, free_cells_[index_distribution_(generator)]);
    pose.x = x;
    pose.y = y;
    pose.theta = theta_distribution_(generator);
    return pose;
  }

  auto get_likelihood_field_as_gridmap() const
  {
    auto message = nav_msgs::msg::OccupancyGrid{};
    message.info = map_metadata_;
    const std::size_t size = map_metadata_.width * map_metadata_.height;
    message.data.resize(size);

    for (std::size_t index = 0; index < size; ++index) {
      message.data[index] = static_cast<std::int8_t>(likelihood_field_[index] * 100);
    }
    return message;
  }

private:
  LikelihoodSensorModelParam params_;
  nav_msgs::msg::MapMetaData map_metadata_;
  std::vector<std::size_t> free_cells_;
  std::uniform_int_distribution<std::size_t> index_distribution_;
  std::uniform_real_distribution<double> theta_distribution_{-M_PI, M_PI};

  std::vector<double> likelihood_field_;

  static std::vector<std::size_t> make_free_cells_list(const nav_msgs::msg::OccupancyGrid & map)
  {
    const std::size_t size = map.info.width * map.info.height;
    auto free_cells = std::vector<std::size_t>{};
    free_cells.reserve(size);
    for (std::size_t index = 0; index < size; ++index) {
      if (map.data[index] == 0) {
        free_cells.push_back(index);
      }
    }
    return free_cells;
  }

  static std::vector<double> make_likelihood_field(
    const LikelihoodSensorModelParam & params,
    const nav_msgs::msg::OccupancyGrid & map)
  {
    auto obstacle_map = map.data | ranges::views::transform(
      [](std::int8_t value) -> bool {
        // Map server occupied output value
        // https://wiki.ros.org/map_server#Value_Interpretation
        return value == 100;
      });

    auto squared_distance =
      [width = map.info.width,
        squared_resolution = map.info.resolution * map.info.resolution,
        squared_max_distance = params.max_obstacle_distance * params.max_obstacle_distance
      ](std::size_t first, std::size_t second) {
        auto delta_x =
          static_cast<std::intmax_t>(first % width) - static_cast<std::intmax_t>(second % width);
        auto delta_y =
          static_cast<std::intmax_t>(first / width) - static_cast<std::intmax_t>(second / width);
        return std::min(
          static_cast<double>(delta_x * delta_x + delta_y * delta_y) * squared_resolution,
          squared_max_distance);
      };

    auto neighbors = [&map](std::size_t index) {
        auto result = std::vector<std::size_t>{};
        if (index / map.info.width < (map.info.height - 1)) {
          result.push_back(index + map.info.width);
        }
        if (index / map.info.width > 0) {
          result.push_back(index - map.info.width);
        }
        if (index % map.info.width < (map.info.width - 1)) {
          result.push_back(index + 1);
        }
        if (index % map.info.width > 0) {
          result.push_back(index - 1);
        }
        return result;
      };

    auto to_likelihood =
      [amplitude = params.z_hit / (params.sigma_hit * std::sqrt(2 * M_PI)),
        two_squared_sigma = 2 * params.sigma_hit * params.sigma_hit,
        offset = params.z_random / params.max_laser_distance](double squared_distance) {
        return amplitude * std::exp(-squared_distance / two_squared_sigma) + offset;
      };

    auto distance_map = nearest_obstacle_distance_map(obstacle_map, squared_distance, neighbors);
    return distance_map | ranges::views::transform(to_likelihood) | ranges::to<std::vector>;
  }

  static auto xy_from_index(
    const nav_msgs::msg::MapMetaData & info,
    std::size_t index)
  {
    return std::make_pair(
      (static_cast<double>(index % info.width) + 0.5) * info.resolution + info.origin.position.x,
      (static_cast<double>(index / info.width) + 0.5) * info.resolution + info.origin.position.y);
  }
};

}  // namespace beluga_amcl

namespace beluga
{

template<>
struct spatial_hash<beluga_amcl::Pose, void>
{
public:
  constexpr std::size_t operator()(
    const beluga_amcl::Pose & pose,
    double resolution = 1.) const
  {
    return spatial_hash<std::tuple<double, double>>{}(
      std::make_tuple(pose.x, pose.y),
      resolution);
  }
};

}  // namespace beluga

#endif  // BELUGA_AMCL__MODELS_HPP_
