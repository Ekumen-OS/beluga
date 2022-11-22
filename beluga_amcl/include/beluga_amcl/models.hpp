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

#ifndef BELUGA_AMCL__MODELS_HPP_
#define BELUGA_AMCL__MODELS_HPP_

#include <beluga/spatial_hash.h>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cmath>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

namespace beluga_amcl
{

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

template<class Mixin>
class MockMotionModel : public Mixin
{
public:
  template<class ... Args>
  explicit MockMotionModel(Args && ... args)
  : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] auto apply_motion(const Pose & state) {return state;}
};

template<class Mixin>
class LikelihoodSensorModel : public Mixin
{
public:
  template<class ... Args>
  explicit LikelihoodSensorModel(const nav_msgs::msg::OccupancyGrid & map, Args && ... rest)
  : Mixin(std::forward<Args>(rest)...)
  {
    const std::size_t size = map.info.width * map.info.height;

    free_cells_.reserve(size);
    for (std::size_t index = 0; index < size; ++index) {
      if (map.data[index] == 0) {
        free_cells_.push_back(index);
      }
    }

    map_metadata_ = map.info;
    index_distribution_ = std::uniform_int_distribution<std::size_t>{0, free_cells_.size() - 1};
  }

  [[nodiscard]] double importance_weight(const Pose &)
  {
    // TODO(nahuel): Implement likelihood estimation
    return 1.;
  }

  template<class Generator>
  [[nodiscard]] auto generate_random_state(Generator & generator)
  {
    return pose_from_index_and_angle(
      map_metadata_,
      free_cells_[index_distribution_(generator)],
      theta_distribution_(generator));
  }

private:
  nav_msgs::msg::MapMetaData map_metadata_;
  std::vector<std::size_t> free_cells_;
  std::uniform_int_distribution<std::size_t> index_distribution_;
  std::uniform_real_distribution<double> theta_distribution_{-M_PI, M_PI};

  static auto pose_from_index_and_angle(
    const nav_msgs::msg::MapMetaData & metadata,
    std::size_t index, double angle)
  {
    double delta_x = (static_cast<double>(index % metadata.height) + 0.5) * metadata.resolution;
    double delta_y = (static_cast<double>(index / metadata.width) + 0.5) * metadata.resolution;
    auto pose = Pose{};
    pose.x = delta_x + metadata.origin.position.x;
    pose.y = delta_y + metadata.origin.position.y;
    pose.theta = angle;
    return pose;
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
    return spatial_hash<std::tuple<double, double>>{} (
      std::make_tuple(pose.x, pose.y),
      resolution);
  }
};

}  // namespace beluga

#endif  // BELUGA_AMCL__MODELS_HPP_
