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

#include "beluga/sensor.hpp"
#include "beluga/test/static_occupancy_grid.hpp"

#include "ciabatta/ciabatta.hpp"

#include <gmock/gmock.h>

namespace {

using beluga::testing::StaticOccupancyGrid;

using UUT = beluga::LikelihoodFieldModel<StaticOccupancyGrid<5, 5>>;

TEST(LikelihoodFieldModel, LikelihoodField) {
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

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto sensor_model = UUT{params, grid};

  // the likelihood field model includes an optimization that stores the likelihood elevated to the cube
  auto expected_cubed_likelihood =
      expected_likelihood_field | ranges::views::transform([](auto v) { return v * v * v; }) | ranges::to<std::vector>;

  ASSERT_THAT(
      sensor_model.likelihood_field().data(),
      testing::Pointwise(testing::DoubleNear(0.003), expected_cubed_likelihood));
}

TEST(LikelihoodFieldModel, ImportanceWeight) {
  constexpr double kResolution = 0.5;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{1.25, 1.25}});
    ASSERT_NEAR(2.068, state_weighting_function(grid.origin()), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{2.25, 2.25}});
    ASSERT_NEAR(1.000, state_weighting_function(grid.origin()), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{-50.0, 50.0}});
    ASSERT_NEAR(1.000, state_weighting_function(grid.origin()), 0.003);
  }

  {
    auto state_weighting_function =
        sensor_model(std::vector<std::pair<double, double>>{{1.20, 1.20}, {1.25, 1.25}, {1.30, 1.30}});
    ASSERT_NEAR(4.205, state_weighting_function(grid.origin()), 0.01);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{0.0, 0.0}});
    ASSERT_NEAR(2.068, state_weighting_function(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{1.25, 1.25}}), 0.003);
  }
}

TEST(LikelihoodFieldModel, GridWithOffset) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, true },
    kResolution,
    Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{-5, -5}}};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{4.5, 4.5}});
    ASSERT_NEAR(2.068, state_weighting_function(Sophus::SE2d{}), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{9.5, 9.5}});
    ASSERT_NEAR(2.068, state_weighting_function(grid.origin()), 0.003);
  }
}

TEST(LikelihoodFieldModel, GridWithRotation) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, true },
    kResolution,
    Sophus::SE2d{Sophus::SO2d{Sophus::Constants<double>::pi() / 2}, Eigen::Vector2d{0.0, 0.0}}};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{-9.5, 9.5}});
    ASSERT_NEAR(2.068, state_weighting_function(Sophus::SE2d{}), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{9.5, 9.5}});
    ASSERT_NEAR(2.068, state_weighting_function(grid.origin()), 0.003);
  }
}

TEST(LikelihoodFieldModel, GridWithRotationAndOffset) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto origin_rotation = Sophus::SO2d{Sophus::Constants<double>::pi() / 2};
  const auto origin = Sophus::SE2d{origin_rotation, origin_rotation * Eigen::Vector2d{-5, -5}};

  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, true },
    kResolution,
    origin};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{-4.5, 4.5}});
    ASSERT_NEAR(2.068, state_weighting_function(Sophus::SE2d{}), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{9.5, 9.5}});
    ASSERT_NEAR(2.068, state_weighting_function(grid.origin()), 0.003);
  }
}

TEST(LikelihoodFieldModel, GridUpdates) {
  const auto origin = Sophus::SE2d{};

  constexpr double kResolution = 0.5;
  // clang-format off
  auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution, origin};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto sensor_model = UUT{params, std::move(grid)};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{1., 1.}});
    EXPECT_NEAR(2.068577607986223, state_weighting_function(origin), 1e-6);
  }

  // clang-format off
  grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, true},
    kResolution, origin};
  // clang-format on
  sensor_model.update_map(std::move(grid));

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{1., 1.}});
    EXPECT_NEAR(1.0, state_weighting_function(origin), 1e-3);
  }
}

}  // namespace
