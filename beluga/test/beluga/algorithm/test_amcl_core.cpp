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

#include <execution>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <sophus/se2.hpp>

#include "beluga/algorithm/amcl_core.hpp"
#include "beluga/algorithm/spatial_hash.hpp"
#include "beluga/containers/tuple_vector.hpp"
#include "beluga/motion/differential_drive_model.hpp"
#include "beluga/sensor/beam_model.hpp"
#include "beluga/sensor/likelihood_field_model.hpp"
#include "beluga/test/static_occupancy_grid.hpp"
#include "beluga/views/particles.hpp"

namespace {

const auto kDummyControl = Sophus::SE2d{};

const std::vector kDummyMeasurement = {
    std::make_pair(0.0, 0.0),
    std::make_pair(0.0, 0.0),
    std::make_pair(0.0, 0.0),
};

auto make_amcl(const beluga::AmclParams& params = {}) {
  constexpr double kResolution = 1.0;
  // clang-format off
  const auto map = beluga::testing::StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false ,
    false, false, false, false , false,
    false, false, true , false, false,
    false, false , false, false, false,
    false , false, false, false, false},
    kResolution};
  // clang-format on
  const beluga::BeamModelParam param{};

  beluga::spatial_hash<Sophus::SE2d> hasher{0.1, 0.1, 0.1};

  beluga::Amcl amcl{
      beluga::DifferentialDriveModel{beluga::DifferentialDriveModelParam{}},  //
      beluga::BeamSensorModel{param, map},                                    //
      std::move(hasher),
      std::move(params),
  };
  return amcl;
}

}  // namespace

namespace beluga {

TEST(TestAmclCore, InitializeWithNoParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
}
TEST(TestAmclCore, Update) {
  auto amcl = make_amcl();
  amcl.update(kDummyControl, kDummyMeasurement);
}

TEST(TestAmclCore, InitializeFromPose) {
  auto amcl = make_amcl();
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), AmclParams{}.max_particles);
}

TEST(TestAmclCore, UpdateWithNoParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  auto estimate = amcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestAmclCore, UpdateWithParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), AmclParams{}.max_particles);
  auto estimate = amcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestAmclCore, UpdateWithParticlesNoMotion) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), AmclParams{}.max_particles);
  auto estimate = amcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
  estimate = amcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestAmclCore, UpdateWithParticlesForced) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), AmclParams{}.max_particles);
  auto estimate = amcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
  amcl.force_update();
  estimate = amcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestAmclCore, SelectiveResampleCanBeConstructed) {
  auto params = beluga::AmclParams{};
  params.selective_resampling = true;
  auto amcl = make_amcl(params);
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), AmclParams{}.max_particles);
  auto estimate = amcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
}

}  // namespace beluga
