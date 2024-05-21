// Copyright 2024 Ekumen, Inc.
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

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

#include <range/v3/algorithm/count_if.hpp>

#include <sophus/common.hpp>
#include <sophus/se2.hpp>

#include "beluga/primitives.hpp"
#include "beluga_ros/messages.hpp"
#include "beluga_ros/particle_cloud.hpp"

namespace {

TEST(TestParticleCloud, Assign) {
  using Constants = Sophus::Constants<double>;
  const auto particles = std::vector{
      std::make_tuple(
          Sophus::SE2d{Sophus::SO2d{-Constants::pi() / 2.0}, Eigen::Vector2d{0.0, 1.0}}, beluga::Weight(0.5)),
      std::make_tuple(
          Sophus::SE2d{Sophus::SO2d{Constants::pi() / 2.0}, Eigen::Vector2d{0.0, -1.0}}, beluga::Weight(0.5)),
  };
  constexpr std::size_t kSampleSize = 1000U;
  auto message = beluga_ros::msg::PoseArray{};
  beluga_ros::assign_particle_cloud(particles, kSampleSize, message);
  EXPECT_EQ(message.poses.size(), kSampleSize);
  for (const auto& pose : message.poses) {
    EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(std::abs(pose.position.y), 1.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(std::abs(pose.orientation.z), std::sin(Constants::pi() / 4.0));
    EXPECT_DOUBLE_EQ(pose.orientation.w, std::cos(Constants::pi() / 4.0));
  }
  const auto num_particles_in_upper_half_xy_plane =
      ranges::count_if(message.poses, [](const auto& pose) { return pose.position.y > 0.0; });
  const double upper_half_xy_plane_weight =
      static_cast<double>(num_particles_in_upper_half_xy_plane) / static_cast<double>(message.poses.size());
  EXPECT_NEAR(upper_half_xy_plane_weight, 0.5, kSampleSize * 0.01);  // allow 1% deviations
}

TEST(TestParticleCloud, AssignNone) {
  const auto particles = std::vector{
      std::make_tuple(Sophus::SE2d{}, beluga::Weight(1.0)),
  };
  auto message = beluga_ros::msg::PoseArray{};
  beluga_ros::assign_particle_cloud(particles, 0U, message);
  EXPECT_EQ(message.poses.size(), 0U);
}

TEST(TestParticleCloud, AssignMatchingDistribution) {
  const auto particles = std::vector{
      std::make_tuple(Sophus::SE2d{}, beluga::Weight(1.0)),
  };
  auto message = beluga_ros::msg::PoseArray{};
  beluga_ros::assign_particle_cloud(particles, message);
  EXPECT_EQ(message.poses.size(), particles.size());
}

TEST(TestParticleCloud, AssignMatchingEmpty) {
  const auto particles = std::vector<std::tuple<Sophus::SE2d, beluga::Weight>>{};
  auto message = beluga_ros::msg::PoseArray{};
  beluga_ros::assign_particle_cloud(particles, message);
  EXPECT_EQ(message.poses.size(), 0U);
}

TEST(TestParticleCloud, AssignMarkers) {
  using Constants = Sophus::Constants<double>;
  const auto particles = std::vector{
      std::make_tuple(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{1.0, 0.0}}, beluga::Weight(0.35)),
      std::make_tuple(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{1.0, 0.0}}, beluga::Weight(0.25)),
      std::make_tuple(Sophus::SE2d{Sophus::SO2d{Constants::pi()}, Eigen::Vector2d{0.0, -1.0}}, beluga::Weight(0.4)),
  };
  auto message = beluga_ros::msg::MarkerArray{};
  beluga_ros::assign_particle_cloud(particles, message);
  ASSERT_EQ(message.markers.size(), 2U);
  EXPECT_EQ(message.markers[0].points.size(), 2 * 2);  // 2 arrows, 2 vertices each
  EXPECT_EQ(message.markers[1].points.size(), 3 * 2);  // 2 arrows, 3 vertices each
}

TEST(TestParticleCloud, AssignNoMarkers) {
  const auto particles = std::vector<std::tuple<Sophus::SE2d, beluga::Weight>>{};
  auto message = beluga_ros::msg::MarkerArray{};
  beluga_ros::assign_particle_cloud(particles, message);
  EXPECT_EQ(message.markers.size(), 0U);
}

}  // namespace
