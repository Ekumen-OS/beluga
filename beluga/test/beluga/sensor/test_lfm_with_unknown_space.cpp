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

using UUT = beluga::LikelihoodFieldModel<StaticOccupancyGrid<5, 5, std::int8_t>>;

TEST(LikelihoodFieldModelUnknownSpace, LikelihoodField) {
  constexpr double kResolution = 0.5;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5, std::int8_t>{{
    -1 , -1 , -1 , 100, 100,
    -1 , 0  , 0  , 0  , 100,
    -1 , 0  , 0  , 0  , 100,
    100, 0  , 0  , 0  , 100,
    100, 100, 100, 100, 100},
    kResolution};
  // clang-format on

  constexpr auto kMaxObstacleDistance = 2.0;
  constexpr auto kParamMaxLaserDistance = 20.0;
  constexpr auto kLaserHitProbability = 0.5;
  constexpr auto kWightForRandomNoise = 0.5;
  constexpr auto kStdForObstaclesBeingHit = 0.2;
  constexpr auto kModelUnknownSpace = true;
  constexpr auto kUnknownSpaceLikelihood = 1 / kParamMaxLaserDistance;

  auto params = beluga::LikelihoodFieldModelParam{kMaxObstacleDistance, kParamMaxLaserDistance,   kLaserHitProbability,
                                                  kWightForRandomNoise, kStdForObstaclesBeingHit, kModelUnknownSpace};
  auto sensor_model = UUT{params, grid};

  // clang-format off
 const double expected_likelihood_field[] = {  // NOLINT(modernize-avoid-c-arrays)
    kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, 1.022, 1.022,
    kUnknownSpaceLikelihood, 0.025                  , 0.027                  , 0.069, 1.022,
    kUnknownSpaceLikelihood, 0.027                  , 0.025                  , 0.069, 1.022,
    1.022                  , 0.069                  , 0.069                  , 0.069, 1.022,
    1.022                  , 1.022                  , 1.022                  , 1.022, 1.022};
  // clang-format on

  ASSERT_THAT(
      sensor_model.likelihood_field().data(),
      testing::Pointwise(testing::DoubleNear(0.003), expected_likelihood_field));
}

TEST(LikelihoodFieldModelUnknownSpace, LikelihoodField2) {
  constexpr double kResolution = 0.5;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5, std::int8_t>{{
    -1, -1, -1, 100, 100,
    -1, -1, -1, 0  , 0  ,
    -1, -1, -1, 0  , 0  ,
    -1, -1, -1, 0  , 0  ,
    -1, -1, -1, 100, 100},
    kResolution};
  // clang-format on

  constexpr auto kMaxObstacleDistance = 2.0;
  constexpr auto kParamMaxLaserDistance = 100.0;
  constexpr auto kLaserHitProbability = 0.5;
  constexpr auto kWightForRandomNoise = 0.5;
  constexpr auto kStdForObstaclesBeingHit = 0.2;
  constexpr auto kModelUnknownSpace = true;
  constexpr auto kUnknownSpaceLikelihood = 1 / kParamMaxLaserDistance;

  auto params = beluga::LikelihoodFieldModelParam{kMaxObstacleDistance, kParamMaxLaserDistance,   kLaserHitProbability,
                                                  kWightForRandomNoise, kStdForObstaclesBeingHit, kModelUnknownSpace};
  auto sensor_model = UUT{params, grid};

  // clang-format off
  const double expected_likelihood_field[] = {  // NOLINT(modernize-avoid-c-arrays)
    kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, 1.002, 1.002,
    kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, 0.049, 0.049,
    kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, 0.005, 0.005,
    kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, 0.049, 0.049,
    kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, kUnknownSpaceLikelihood, 1.002, 1.002,
  };
  // clang-format on

  ASSERT_THAT(
      sensor_model.likelihood_field().data(),
      testing::Pointwise(testing::DoubleNear(0.003), expected_likelihood_field));
}

TEST(LikelihoodFieldModelUnknownSpace, ImportanceWeight) {
  constexpr double kResolution = 0.5;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5, std::int8_t>{{
    -1, -1, -1 , 0 , 0 ,
    -1, 0 , 0  , 0 , 0 ,
    -1, 0 , 100, 0 , -1,
    0 , 0 , 0  , 0 , -1,
    0 , 0 , -1 , -1, -1},
    kResolution};
  // clang-format on

  constexpr auto kMaxObstacleDistance = 2.0;
  constexpr auto kParamMaxLaserDistance = 20.0;
  constexpr auto kLaserHitProbability = 0.5;
  constexpr auto kWightForRandomNoise = 0.5;
  constexpr auto kStdForObstaclesBeingHit = 0.2;
  constexpr auto kModelUnknownSpace = true;
  constexpr auto kUnknownSpaceLikelihood = 1 / kParamMaxLaserDistance;

  auto params = beluga::LikelihoodFieldModelParam{kMaxObstacleDistance, kParamMaxLaserDistance,   kLaserHitProbability,
                                                  kWightForRandomNoise, kStdForObstaclesBeingHit, kModelUnknownSpace};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{0, 0}});
    ASSERT_NEAR(
        1.000 + kUnknownSpaceLikelihood * kUnknownSpaceLikelihood * kUnknownSpaceLikelihood,
        state_weighting_function(grid.origin()), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{1.25, 1.25}});
    ASSERT_NEAR(2.068, state_weighting_function(grid.origin()), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{2.25, 2.25}});
    ASSERT_NEAR(
        1.000 + kUnknownSpaceLikelihood * kUnknownSpaceLikelihood * kUnknownSpaceLikelihood,
        state_weighting_function(grid.origin()), 0.003);
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

TEST(LikelihoodFieldModelUnknownSpace, GridWithOffset) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5, std::int8_t>{{
    -1, -1, -1, 0, 0  ,
    -1, 0 , 0 , 0, 0  ,
    -1, 0 , 0 , 0, 0  ,
    0 , 0 , 0 , 0, 0  ,
    0 , 0 , 0 , 0, 100},
    kResolution,
    Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{-5, -5}}};
  // clang-format on

  constexpr auto kMaxObstacleDistance = 2.0;
  constexpr auto kParamMaxLaserDistance = 20.0;
  constexpr auto kLaserHitProbability = 0.5;
  constexpr auto kWightForRandomNoise = 0.5;
  constexpr auto kStdForObstaclesBeingHit = 0.2;
  constexpr auto kModelUnknownSpace = true;

  auto params = beluga::LikelihoodFieldModelParam{kMaxObstacleDistance, kParamMaxLaserDistance,   kLaserHitProbability,
                                                  kWightForRandomNoise, kStdForObstaclesBeingHit, kModelUnknownSpace};
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

TEST(LikelihoodFieldModelUnknownSpace, GridWithRotation) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5, std::int8_t>{{
    -1, -1, -1, 0, 0  ,
    -1, 0 , 0 , 0, 0  ,
    -1, 0 , 0 , 0, 0  ,
    0 , 0 , 0 , 0, 0  ,
    0 , 0 , 0 , 0, 100},
    kResolution,
    Sophus::SE2d{Sophus::SO2d{Sophus::Constants<double>::pi() / 2}, Eigen::Vector2d{0.0, 0.0}}};
  // clang-format on

  constexpr auto kMaxObstacleDistance = 2.0;
  constexpr auto kParamMaxLaserDistance = 20.0;
  constexpr auto kLaserHitProbability = 0.5;
  constexpr auto kWightForRandomNoise = 0.5;
  constexpr auto kStdForObstaclesBeingHit = 0.2;
  constexpr auto kModelUnknownSpace = true;

  auto params = beluga::LikelihoodFieldModelParam{kMaxObstacleDistance, kParamMaxLaserDistance,   kLaserHitProbability,
                                                  kWightForRandomNoise, kStdForObstaclesBeingHit, kModelUnknownSpace};
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

TEST(LikelihoodFieldModelUnknownSpace, GridWithRotationAndOffset) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto origin_rotation = Sophus::SO2d{Sophus::Constants<double>::pi() / 2};
  const auto origin = Sophus::SE2d{origin_rotation, origin_rotation * Eigen::Vector2d{-5, -5}};

  const auto grid = StaticOccupancyGrid<5, 5, std::int8_t>{{
    -1, -1, -1, 0, 0  ,
    -1, 0 , 0 , 0, 0  ,
    -1, 0 , 0 , 0, 0  ,
    0 , 0 , 0 , 0, 0  ,
    0 , 0 , 0 , 0, 100},
    kResolution,
    origin};
  // clang-format on

  constexpr auto kMaxObstacleDistance = 2.0;
  constexpr auto kParamMaxLaserDistance = 20.0;
  constexpr auto kLaserHitProbability = 0.5;
  constexpr auto kWightForRandomNoise = 0.5;
  constexpr auto kStdForObstaclesBeingHit = 0.2;
  constexpr auto kModelUnknownSpace = true;

  auto params = beluga::LikelihoodFieldModelParam{kMaxObstacleDistance, kParamMaxLaserDistance,   kLaserHitProbability,
                                                  kWightForRandomNoise, kStdForObstaclesBeingHit, kModelUnknownSpace};
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

TEST(LikelihoodFieldModelUnknownSpace, GridUpdates) {
  const auto origin = Sophus::SE2d{};

  constexpr double kResolution = 0.5;
  // clang-format off
  auto grid = StaticOccupancyGrid<5, 5, std::int8_t>{{
    -1, -1, -1, 0, 0,
    -1, 0 , 0 , 0, 0,
    -1, 0 , 0 , 0, 0,
    0 , 0 , 0 , 0, 0,
    0 , 0 , 0 , 0, 0},
    kResolution, origin};
  // clang-format on

  constexpr auto kMaxObstacleDistance = 2.0;
  constexpr auto kParamMaxLaserDistance = 20.0;
  constexpr auto kLaserHitProbability = 0.5;
  constexpr auto kWightForRandomNoise = 0.5;
  constexpr auto kStdForObstaclesBeingHit = 0.2;
  constexpr auto kModelUnknownSpace = true;

  auto params = beluga::LikelihoodFieldModelParam{kMaxObstacleDistance, kParamMaxLaserDistance,   kLaserHitProbability,
                                                  kWightForRandomNoise, kStdForObstaclesBeingHit, kModelUnknownSpace};
  auto sensor_model = UUT{params, std::move(grid)};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{1., 1.}});
    EXPECT_NEAR(2.068577607986223, state_weighting_function(origin), 1e-6);
  }

  // clang-format off
  grid = StaticOccupancyGrid<5, 5, std::int8_t>{{
    -1, -1, -1, 0, 0  ,
    -1, 0 , 0 , 0, 0  ,
    -1, 0 , 0 , 0, 0  ,
    0 , 0 , 0 , 0, 0  ,
    0 , 0 , 0 , 0, 100},
    kResolution, origin};
  // clang-format on
  sensor_model.update_map(std::move(grid));

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{1., 1.}});
    EXPECT_NEAR(1.0, state_weighting_function(origin), 1e-3);
  }
}

}  // namespace
