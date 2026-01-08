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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/common.hpp>

#include "beluga/sensor/likelihood_field_model.hpp"
#include "beluga/test/static_occupancy_grid.hpp"

namespace {

using beluga::testing::StaticOccupancyGrid;

using UUT = beluga::LikelihoodFieldModelBase<StaticOccupancyGrid<5, 5>>;

TEST(LikelihoodFieldModelBase, LikelihoodField) {
  constexpr double kResolution = 0.5;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, true ,
    false, false, false, true , false,
    false, false, true , false, false,
    false, true , false, false, false,
    true , false, false, false, false},
    kResolution};

  const double expected_likelihood_field[] = {  // NOLINT(modernize-avoid-c-arrays)
    0.025, 0.025, 0.025, 0.069, 1.022,
    0.025, 0.027, 0.069, 1.022, 0.069,
    0.025, 0.069, 1.022, 0.069, 0.025,
    0.069, 1.022, 0.069, 0.027, 0.025,
    1.022, 0.069, 0.025, 0.025, 0.025
  };
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelBaseParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto sensor_model = UUT{params, grid};

  ASSERT_THAT(
      sensor_model.likelihood_field().data(),
      testing::Pointwise(testing::DoubleNear(0.003), expected_likelihood_field));
}

TEST(LikelihoodFieldModelBase, PreProcessThickWalls) {
  // Create a 5x5 grid with a 3x3 thick wall of obstacles in the center.
  // The grid has a resolution of 1.0 m.
  // F F F F F
  // F O O O F
  // F O O O F
  // F O O O F
  // F F F F F
  const auto grid = beluga::testing::StaticOccupancyGrid<5, 5>{
      {false, false, false, false, false,  //
       false, true,  true,  true,  false,  //
       false, true,  true,  true,  false,  //
       false, true,  true,  true,  false,  //
       false, false, false, false, false},
      1.0};

  auto params = beluga::LikelihoodFieldModelBaseParam{};
  params.likelihood_from_strict_obstacle_edges = true;
  params.max_obstacle_distance = 10.0;
  params.sigma_hit = 0.2;
  params.z_hit = 0.5;
  params.z_random = 0.5;
  params.max_laser_distance = 2.0;

  auto model = UUT{params, grid};

  const auto& likelihood_field = model.likelihood_field();

  // With likelihood_from_strict_obstacle_edges=true, the center obstacle cell (12) at (2,2)
  // should be treated as unknown, and its distance should be calculated
  // from the boundary cells (e.g., cell 7 at (2,1)), which is 1.0 m.
  // The boundary cells themselves should have a distance of 0.

  const double two_squared_sigma = 2 * params.sigma_hit * params.sigma_hit;
  const double amplitude = params.z_hit / (params.sigma_hit * std::sqrt(2 * Sophus::Constants<double>::pi()));
  const double offset = params.z_random / params.max_laser_distance;

  const auto to_likelihood = [=](double squared_distance) {
    return amplitude * std::exp(-squared_distance / two_squared_sigma) + offset;
  };

  // Cell at (0,0) is free. Distance to nearest obstacle (cell 6 at (1,1)) is sqrt(2).
  ASSERT_NEAR(likelihood_field.data_at(0, 0).value(), to_likelihood(2.0), 1e-6);
  // Cell at (1,1) is a boundary obstacle. Distance should be 0.
  ASSERT_NEAR(likelihood_field.data_at(1, 1).value(), to_likelihood(0.0), 1e-6);
  // Cell at (2,2) is an inner obstacle. It should be treated as free space, and its distance to the boundary is 1.0.
  ASSERT_NEAR(likelihood_field.data_at(2, 2).value(), to_likelihood(1.0), 1e-6);
}

}  // namespace
