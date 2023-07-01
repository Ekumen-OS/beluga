// Copyright 2023 Ekumen, Inc.
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

#include "beluga/algorithm/raycasting.hpp"
#include "beluga/sensor.hpp"
#include "beluga/sensor/beam_model.hpp"

#include "beluga/test/static_occupancy_grid.hpp"

#include "ciabatta/ciabatta.hpp"

namespace beluga {

using beluga::testing::StaticOccupancyGrid;

using UUT = ciabatta::mixin<
    ciabatta::curry<beluga::BeamSensorModel, StaticOccupancyGrid>::mixin,
    ciabatta::provides<beluga::LaserSensorModelInterface2d<StaticOccupancyGrid>>::mixin>;

BeamModelParam GetParams() {
  BeamModelParam ret;
  ret.z_hit = 0.5;
  ret.z_short = 0.05;
  ret.z_max = 0.05;
  ret.z_rand = 0.5;
  ret.sigma_hit = 0.2;
  ret.lambda_short = 0.1;
  ret.beam_max_range = 60;
  return ret;
}
TEST(BeamSensorModel, ImportanceWeight) {
  constexpr double kResolution = 0.5;
  // clang-format off
  auto grid_storage = StaticOccupancyGrid::MapStorage(5, 5 ,{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false});
  // clang-format on
  const auto grid = StaticOccupancyGrid(std::move(grid_storage), kResolution, Sophus::SE2d{});

  const auto params = GetParams();
  auto mixin = UUT{params, grid};

  // Perfect hit.
  mixin.update_sensor(std::vector<std::pair<double, double>>{{1., 1.}});
  EXPECT_NEAR(1.0171643824743635, mixin.importance_weight(grid.origin()), 1e-6);

  // This is a hit that's before the obstacle, hence is affected by the unexpected obstacle part of the distribution.
  mixin.update_sensor(std::vector<std::pair<double, double>>{{0.75, 0.75}});
  EXPECT_NEAR(0.015905891701088148, mixin.importance_weight(grid.origin()), 1e-6);

  // Hit that's past the obstacle, hence is not affected by the unexpected obstacle part of the distribution.
  // This should be really close to zero.
  mixin.update_sensor(std::vector<std::pair<double, double>>{{2.25, 2.25}});
  EXPECT_NEAR(0.000, mixin.importance_weight(grid.origin()), 1e-6);

  // Range return longer than beam_max_range, so the max measurement distribution kicks in and this shouldn't be
  // zero.
  mixin.update_sensor(std::vector<std::pair<double, double>>{{params.beam_max_range, params.beam_max_range}});
  EXPECT_NEAR(0.00012500000000000003, mixin.importance_weight(grid.origin()), 1e-6);
}

TEST(BeamSensorModel, GridUpdates) {
  const auto origin = Sophus::SE2d{};

  constexpr double kResolution = 0.5;
  // clang-format off
  auto grid_storage = StaticOccupancyGrid::MapStorage(5, 5 ,{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false});
  // clang-format on
  const auto grid = StaticOccupancyGrid(std::move(grid_storage), kResolution, origin);

  const auto params = GetParams();
  auto mixin = UUT{params, std::move(grid)};

  mixin.update_sensor(std::vector<std::pair<double, double>>{{1., 1.}});
  EXPECT_NEAR(1.0171643824743635, mixin.importance_weight(origin), 1e-6);

  // clang-format off
  auto new_grid_storage = StaticOccupancyGrid::MapStorage(5, 5 ,{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false});
  // clang-format on
  auto new_grid = StaticOccupancyGrid(std::move(new_grid_storage), kResolution, origin);

  mixin.update_map(std::move(new_grid));

  mixin.update_sensor(std::vector<std::pair<double, double>>{{1., 1.}});
  EXPECT_NEAR(0.0, mixin.importance_weight(origin), 1e-3);
}

}  // namespace beluga
