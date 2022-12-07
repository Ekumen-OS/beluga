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

#include <tf2/convert.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <queue>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <range/v3/view.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
class StationaryMotionModel : public Mixin
{
public:
  template<class ... Args>
  explicit StationaryMotionModel(Args && ... args)
  : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] auto apply_motion(const Pose & state) const
  {
    static thread_local std::mt19937 generator{std::random_device()()};
    auto distribution = std::normal_distribution<>{0, 0.02};
    auto pose = state;
    pose.x += distribution(generator);
    pose.y += distribution(generator);
    pose.theta = wrap_angle(pose.theta + distribution(generator));
    return pose;
  }

private:
  static double wrap_angle(double angle)
  {
    // Wrap angle between -pi and pi
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    return angle <= 0.0 ? angle + M_PI : angle - M_PI;
  }
};

struct LikelihoodSensorModelParam
{
  double max_obstacle_distance;
  double min_laser_distance;
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
    likelihood_field_{make_likelihood_field(params, map)}
  {}

  [[nodiscard]] double importance_weight(const Pose & pose) const
  {
    // Compute offset in scaled map coordinates
    const auto x_offset = (pose.x - map_metadata_.origin.position.x) / map_metadata_.resolution;
    const auto y_offset = (pose.y - map_metadata_.origin.position.y) / map_metadata_.resolution;
    const auto lock = std::shared_lock<std::shared_mutex>(points_mutex_);
    /* *INDENT-OFF* Avoid uncrustify reformat */
    return std::transform_reduce(
      points_.cbegin(), points_.cend(), 0.0, std::plus{},
      [this, pose, x_offset, y_offset](const auto & point) {
        const auto x = point.first * std::cos(pose.theta) - point.second * std::sin(pose.theta) + x_offset;
        const auto y = point.first * std::sin(pose.theta) + point.second * std::cos(pose.theta) + y_offset;
        const auto x_index = static_cast<std::intmax_t>(std::floor(x));
        const auto y_index = static_cast<std::intmax_t>(std::floor(y));
        if (x_index >= map_metadata_.width || y_index >= map_metadata_.height) {
          return 0.;
        }
        return likelihood_field_[x_index + y_index * map_metadata_.width];
      });
    /* *INDENT-ON* */
  }

  template<class Generator>
  [[nodiscard]] auto generate_random_state(Generator & generator) const
  {
    auto index_distribution = std::uniform_int_distribution<std::size_t>{0, free_cells_.size() - 1};
    auto theta_distribution = std::uniform_real_distribution<double>{-M_PI, M_PI};
    auto pose = Pose{};
    const auto index = free_cells_[index_distribution(generator)];
    pose.x = static_cast<double>(index % map_metadata_.width) + 0.5;
    pose.y = static_cast<double>(index / map_metadata_.width) + 0.5;
    pose.x = pose.x * map_metadata_.resolution + map_metadata_.origin.position.x;
    pose.y = pose.y * map_metadata_.resolution + map_metadata_.origin.position.y;
    pose.theta = theta_distribution(generator);
    return pose;
  }

  void update_sensor(
    const sensor_msgs::msg::LaserScan & laser_scan,
    const geometry_msgs::msg::TransformStamped & base_to_laser_stamped)
  {
    auto base_to_laser_transform = tf2::Transform{};
    tf2::convert(base_to_laser_stamped.transform, base_to_laser_transform);
    const float range_min =
      std::max(laser_scan.range_min, static_cast<float>(params_.min_laser_distance));
    const float range_max =
      std::min(laser_scan.range_max, static_cast<float>(params_.max_laser_distance));
    auto point_buffer = decltype(points_) {};
    point_buffer.reserve(laser_scan.ranges.size());
    for (std::size_t index = 0; index < laser_scan.ranges.size(); ++index) {
      float range = laser_scan.ranges[index];
      if (std::isnan(range) || range < range_min || range > range_max) {
        continue;
      }
      // Store points in the robot's reference frame with scaled coordinates
      const float angle = laser_scan.angle_min +
        static_cast<float>(index) * laser_scan.angle_increment;
      const auto point = base_to_laser_transform * tf2::Vector3{
        range * std::cos(angle),
        range * std::sin(angle),
        0.0};
      point_buffer.emplace_back(
        point.x() / map_metadata_.resolution,
        point.y() / map_metadata_.resolution);
    }
    const auto lock = std::lock_guard<std::shared_mutex>(points_mutex_);
    points_ = std::move(point_buffer);
  }

  auto get_likelihood_field_as_gridmap() const
  {
    const std::size_t size = map_metadata_.width * map_metadata_.height;
    auto message = nav_msgs::msg::OccupancyGrid{};
    message.info = map_metadata_;
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
  std::vector<double> likelihood_field_;

  // TODO(nahuel): Create a mixin type to ensure thread safety for sensor and motion models.
  std::vector<std::pair<double, double>> points_;
  mutable std::shared_mutex points_mutex_;

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
        const auto delta_x =
          static_cast<std::intmax_t>(first % width) - static_cast<std::intmax_t>(second % width);
        const auto delta_y =
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

    const auto distance_map = nearest_obstacle_distance_map(
      obstacle_map, squared_distance, neighbors);
    return distance_map | ranges::views::transform(to_likelihood) | ranges::to<std::vector>;
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
