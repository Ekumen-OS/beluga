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

#include "beluga/algorithm/mcl_core.hpp"
#include "beluga/containers/tuple_vector.hpp"
#include "beluga/motion/differential_drive_model.hpp"
#include "beluga/sensor/beam_model.hpp"
#include "beluga/test/static_occupancy_grid.hpp"
#include "beluga/views/particles.hpp"

namespace {

const auto kDummyControl = Sophus::SE2d{};
const std::vector kDummyMeasurement = {
    std::make_pair(0.0, 0.0),
    std::make_pair(0.0, 0.0),
    std::make_pair(0.0, 0.0),
};

auto make_mcl(const beluga::MclParams& params = {}) {
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

  beluga::Mcl mcl{
      beluga::DifferentialDriveModel{beluga::DifferentialDriveModelParam{}},  //
      beluga::BeamSensorModel{param, map},                                    //
      std::move(params),                                                      //
      std::execution::seq,
  };
  return mcl;
}
}  // namespace

namespace beluga {
TEST(TestMclCore, InitializeWithNoParticles) {
  auto mcl = make_mcl();
  ASSERT_EQ(mcl.particles().size(), 0);
}
TEST(TestMclCore, Update) {
  auto mcl = make_mcl();
  mcl.update(kDummyControl, kDummyMeasurement);
}

TEST(TestMclCore, InitializeFromPose) {
  auto mcl = make_mcl();
  mcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(mcl.particles().size(), MclParams{}.num_particles);
}

TEST(TestMclCore, UpdateWithNoParticles) {
  auto mcl = make_mcl();
  ASSERT_EQ(mcl.particles().size(), 0);
  auto estimate = mcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestMclCore, UpdateWithParticles) {
  auto mcl = make_mcl();
  ASSERT_EQ(mcl.particles().size(), 0);
  mcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(mcl.particles().size(), MclParams{}.num_particles);
  auto estimate = mcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestMclCore, UpdateWithParticlesNoMotion) {
  auto mcl = make_mcl();
  ASSERT_EQ(mcl.particles().size(), 0);
  mcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(mcl.particles().size(), MclParams{}.num_particles);
  auto estimate = mcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
  estimate = mcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestMclCore, UpdateWithParticlesForced) {
  auto mcl = make_mcl();
  ASSERT_EQ(mcl.particles().size(), 0);
  mcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(mcl.particles().size(), MclParams{}.num_particles);
  auto estimate = mcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
  mcl.force_update();
  estimate = mcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestMclCore, SelectiveResampleCanBeConstructed) {
  auto params = beluga::MclParams{};
  params.selective_resampling = true;
  auto mcl = make_mcl(params);
  mcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(mcl.particles().size(), MclParams{}.num_particles);
  auto estimate = mcl.update(kDummyControl, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
}

}  // namespace beluga
