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

#include "beluga/algorithm/amcl_core.hpp"
#include "beluga/algorithm/spatial_hash.hpp"
#include "beluga/containers/tuple_vector.hpp"
#include "beluga/test/static_occupancy_grid.hpp"
#include "beluga/views/particles.hpp"

#include <gmock/gmock.h>
#include <range/v3/view/take_last.hpp>
#include <sophus/se2.hpp>

using beluga::testing::StaticOccupancyGrid;
namespace beluga {

const std::vector kDummyMeasurement = {
    std::make_pair(1.0, 2.0),
    std::make_pair(1.2, 2.0),
    std::make_pair(1.3, 2.0),
};

auto make_amcl() {
  auto params = beluga::AmclParams{};
  params.max_particles = 50UL;

  constexpr double kResolution = 0.5;
  // clang-format off
  const auto map = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, true ,
    false, false, false, true , false,
    false, false, true , false, false,
    false, true , false, false, false,
    true , false, false, false, false},
    kResolution};

    auto random_state_maker = [](){
        return Sophus::SE2d{};
    };

    beluga::spatial_hash<Sophus::SE2d> hasher{0.5, 0.5, 0.5};

    beluga::Amcl amcl{
      beluga::DifferentialDriveModel{beluga::DifferentialDriveModelParam{}},   //
      beluga::LikelihoodFieldModel{beluga::LikelihoodFieldModelParam{}, map},  //
      std::move(random_state_maker),
      std::move(hasher),
      std::move(params),                                                                  //
      std::execution::seq,
  };
  return amcl;
}

TEST(TestAmclCore, InitializeWithNoParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
}
TEST(TestAmclCore, Update) {
  auto amcl = make_amcl();
  amcl.update(Sophus::SE2d{}, kDummyMeasurement);
}

TEST(TestAmclCore, InitializeFromPose) {
  auto amcl = make_amcl();
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), 50UL);
}

TEST(TestAmclCore, UpdateWithNoParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  auto estimate = amcl.update(Sophus::SE2d{}, kDummyMeasurement);
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestAmclCore, UpdateWithParticles) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
}

TEST(TestAmclCore, UpdateWithParticlesNoMotion) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
  estimate = amcl.update(Sophus::SE2d{}, kDummyMeasurement);
  ASSERT_FALSE(estimate.has_value());
}

TEST(TestAmclCore, UpdateWithParticlesForced) {
  auto amcl = make_amcl();
  ASSERT_EQ(amcl.particles().size(), 0);
  amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
  ASSERT_EQ(amcl.particles().size(), 50UL);
  auto estimate = amcl.update(Sophus::SE2d{}, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
  amcl.force_update();
  estimate = amcl.update(Sophus::SE2d{}, kDummyMeasurement);
  ASSERT_TRUE(estimate.has_value());
}


TEST(TestAmclCore, ParticlesDependentRandomStateGenerator) {
  auto params = beluga::AmclParams{};
  params.max_particles = 50UL;

  constexpr double kResolution = 0.5;
  // clang-format off
  const auto map = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, true ,
    false, false, false, true , false,
    false, false, true , false, false,
    false, true , false, false, false,
    true , false, false, false, false},
    kResolution};

    // Demonstrate how we can provide a generator that depends on the current filter state.
    auto random_state_maker = [](const auto& particles){
        const auto last_particle_state = beluga::views::states(particles).back();
        return [last_particle_state](){return last_particle_state;};
    };

    beluga::spatial_hash<Sophus::SE2d> hasher{0.5, 0.5, 0.5};

    beluga::Amcl amcl{
      beluga::DifferentialDriveModel{beluga::DifferentialDriveModelParam{}},   //
      beluga::LikelihoodFieldModel{beluga::LikelihoodFieldModelParam{}, map},  //
      std::move(random_state_maker),
      std::move(hasher),
      std::move(params),                                                                  //
      std::execution::seq,
    };
    amcl.initialize(Sophus::SE2d{}, Eigen::Vector3d::Ones().asDiagonal());
    ASSERT_EQ(amcl.particles().size(), 50UL);
    auto estimate = amcl.update(Sophus::SE2d{}, kDummyMeasurement);
    ASSERT_TRUE(estimate.has_value());
}

}  // namespace beluga
