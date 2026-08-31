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

#include "beluga/sensor/likelihood_field_prob_model.hpp"
#include "beluga/test/static_occupancy_grid.hpp"

namespace {

using beluga::testing::StaticOccupancyGrid;

using UUT = beluga::LikelihoodFieldProbModel<StaticOccupancyGrid<5, 5>>;

TEST(LikelihoodFieldProbModel, ImportanceWeight) {
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

  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{1.25, 1.25}});
    ASSERT_NEAR(1.022, state_weighting_function(grid.origin()), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{2.25, 2.25}});
    ASSERT_NEAR(0.025, state_weighting_function(grid.origin()), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{-50.0, 50.0}});
    ASSERT_NEAR(0.050, state_weighting_function(grid.origin()), 0.003);
  }

  {
    auto state_weighting_function =
        sensor_model(std::vector<std::pair<double, double>>{{1.20, 1.20}, {1.25, 1.25}, {1.30, 1.30}});
    ASSERT_NEAR(1.068, state_weighting_function(grid.origin()), 0.01);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{0.0, 0.0}});
    ASSERT_NEAR(1.022, state_weighting_function(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{1.25, 1.25}}), 0.003);
  }
}

TEST(LikelihoodFieldProbModel, GridWithOffset) {
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

  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{4.5, 4.5}});
    ASSERT_NEAR(1.022, state_weighting_function(Sophus::SE2d{}), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{9.5, 9.5}});
    ASSERT_NEAR(1.022, state_weighting_function(grid.origin()), 0.003);
  }
}

TEST(LikelihoodFieldProbModel, GridWithRotation) {
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

  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{-9.5, 9.5}});
    ASSERT_NEAR(1.022, state_weighting_function(Sophus::SE2d{}), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{9.5, 9.5}});
    ASSERT_NEAR(1.022, state_weighting_function(grid.origin()), 0.003);
  }
}

TEST(LikelihoodFieldProbModel, GridWithRotationAndOffset) {
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

  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}};
  auto sensor_model = UUT{params, grid};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{-4.5, 4.5}});
    ASSERT_NEAR(1.022, state_weighting_function(Sophus::SE2d{}), 0.003);
  }

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{9.5, 9.5}});
    ASSERT_NEAR(1.022, state_weighting_function(grid.origin()), 0.003);
  }
}

TEST(LikelihoodFieldProbModel, GridUpdates) {
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

  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}};
  auto sensor_model = UUT{params, std::move(grid)};

  {
    auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{{1., 1.}});
    EXPECT_NEAR(1.0223556756973267, state_weighting_function(origin), 1e-6);
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
    EXPECT_NEAR(0.025, state_weighting_function(origin), 1e-3);
  }
}

// Builds the standard 5x5 grid used by the beam skipping tests: a single obstacle at the center
// cell, so that the beam {1.25, 1.25} lands exactly on it (pz ~ 1.022) and beams further away
// floor at pz = z_random / max_laser_distance = 0.025.
StaticOccupancyGrid<5, 5> make_beamskip_grid() {
  constexpr double kResolution = 0.5;
  // clang-format off
  return StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution};
  // clang-format on
}

TEST(LikelihoodFieldProbModelBeamSkip, DisabledMatchesBaseline) {
  const auto grid = make_beamskip_grid();
  // do_beamskip defaults to false when omitted from the aggregate initializer.
  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}};
  auto sensor_model = UUT{params, grid};

  // prepare() is a no-op while skipping is disabled: the mask stays empty and weights are unchanged.
  const auto points = std::vector<std::pair<double, double>>{{1.25, 1.25}, {2.25, 2.25}};
  sensor_model.prepare(points, std::vector<Sophus::SE2d>(10, grid.origin()));
  EXPECT_TRUE(sensor_model.beam_mask().empty());

  auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{points});
  // Both beams contribute: 1.022 (obstacle) * 0.025 (floor).
  ASSERT_NEAR(1.022 * 0.025, state_weighting_function(grid.origin()), 0.003 * 0.025);
}

TEST(LikelihoodFieldProbModelBeamSkip, ExcludesDivergentBeam) {
  const auto grid = make_beamskip_grid();
  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}, true, 0.5, 0.3, 0.9};
  auto sensor_model = UUT{params, grid};

  // One beam hits the obstacle (agrees with the map), the other consistently misses it
  // (simulated dynamic obstacle).
  const auto points = std::vector<std::pair<double, double>>{{1.25, 1.25}, {2.25, 2.25}};
  sensor_model.prepare(points, std::vector<Sophus::SE2d>(10, grid.origin()));

  ASSERT_EQ(sensor_model.beam_mask().size(), 2U);
  EXPECT_TRUE(sensor_model.beam_mask()[0]);   // obstacle beam is kept
  EXPECT_FALSE(sensor_model.beam_mask()[1]);  // divergent beam is skipped

  // With the divergent beam skipped, only the obstacle beam contributes (~1.022), which is higher
  // than the full product 1.022 * 0.025 the model would yield without skipping.
  auto state_weighting_function = sensor_model(std::vector<std::pair<double, double>>{points});
  ASSERT_NEAR(1.022, state_weighting_function(grid.origin()), 0.003);
}

TEST(LikelihoodFieldProbModelBeamSkip, KeepsAgreedBeam) {
  const auto grid = make_beamskip_grid();
  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}, true, 0.5, 0.3, 0.9};
  auto sensor_model = UUT{params, grid};

  // 7 of 10 particles agree on the obstacle beam (0.7 > beam_skip_threshold), so it is kept.
  auto states = std::vector<Sophus::SE2d>(7, grid.origin());
  states.resize(10, Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{10., 10.}});  // 3 particles miss
  sensor_model.prepare(std::vector<std::pair<double, double>>{{1.25, 1.25}}, states);

  ASSERT_EQ(sensor_model.beam_mask().size(), 1U);
  EXPECT_TRUE(sensor_model.beam_mask()[0]);
}

TEST(LikelihoodFieldProbModelBeamSkip, ThresholdBoundary) {
  const auto grid = make_beamskip_grid();
  // beam_skip_error_threshold = 1.0 so the single-beam case is decided purely by beam_skip_threshold
  // and never triggers the all-beams-skipped fallback.
  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}, true, 0.5, 0.3, 1.0};
  auto sensor_model = UUT{params, grid};

  const auto agreeing = grid.origin();
  const auto missing = Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{10., 10.}};
  const auto points = std::vector<std::pair<double, double>>{{1.25, 1.25}};

  // 4/10 = 0.4 > 0.3 -> beam is kept.
  {
    auto states = std::vector<Sophus::SE2d>(4, agreeing);
    states.resize(10, missing);
    sensor_model.prepare(points, states);
    ASSERT_EQ(sensor_model.beam_mask().size(), 1U);
    EXPECT_TRUE(sensor_model.beam_mask()[0]);
  }

  // 2/10 = 0.2 < 0.3 -> beam is skipped.
  {
    auto states = std::vector<Sophus::SE2d>(2, agreeing);
    states.resize(10, missing);
    sensor_model.prepare(points, states);
    ASSERT_EQ(sensor_model.beam_mask().size(), 1U);
    EXPECT_FALSE(sensor_model.beam_mask()[0]);
  }
}

TEST(LikelihoodFieldProbModelBeamSkip, ErrorThresholdFallback) {
  const auto grid = make_beamskip_grid();
  // All beams miss the map, so all would be skipped; that exceeds beam_skip_error_threshold (0.9)
  // and the heuristic falls back to using every beam.
  const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}, true, 0.5, 0.3, 0.9};
  auto sensor_model = UUT{params, grid};

  const auto points = std::vector<std::pair<double, double>>{{2.25, 2.25}, {2.30, 2.30}, {2.35, 2.35}};
  sensor_model.prepare(points, std::vector<Sophus::SE2d>(10, grid.origin()));

  ASSERT_EQ(sensor_model.beam_mask().size(), 3U);
  EXPECT_TRUE(sensor_model.beam_mask()[0]);
  EXPECT_TRUE(sensor_model.beam_mask()[1]);
  EXPECT_TRUE(sensor_model.beam_mask()[2]);

  // The weight matches the model with skipping disabled (all beams integrated).
  const auto baseline_params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}};
  auto baseline_model = UUT{baseline_params, grid};

  const auto weight_with_fallback = sensor_model(std::vector<std::pair<double, double>>{points})(grid.origin());
  const auto weight_baseline = baseline_model(std::vector<std::pair<double, double>>{points})(grid.origin());
  EXPECT_NEAR(weight_with_fallback, weight_baseline, 1e-9);
}

TEST(LikelihoodFieldProbModelBeamSkip, DistanceToLikelihoodThreshold) {
  const auto grid = make_beamskip_grid();
  // Beam one cell (0.5 m) away from the obstacle, so its likelihood (~0.069) sits between the
  // thresholds produced by the two beam_skip_distance values below.
  const auto points = std::vector<std::pair<double, double>>{{1.75, 1.25}};
  const auto states = std::vector<Sophus::SE2d>(10, grid.origin());
  // beam_skip_error_threshold = 1.0 keeps the single-beam decision tied to the distance threshold.

  // Small beam_skip_distance -> high likelihood threshold -> the beam does not agree -> skipped.
  {
    const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}, true, 0.3, 0.3, 1.0};
    auto sensor_model = UUT{params, grid};
    sensor_model.prepare(points, states);
    ASSERT_EQ(sensor_model.beam_mask().size(), 1U);
    EXPECT_FALSE(sensor_model.beam_mask()[0]);
  }

  // Larger beam_skip_distance -> lower likelihood threshold -> the beam agrees -> kept.
  {
    const auto params = beluga::LikelihoodFieldProbModelParam{{2.0, 20.0, 0.5, 0.5, 0.2}, true, 0.8, 0.3, 1.0};
    auto sensor_model = UUT{params, grid};
    sensor_model.prepare(points, states);
    ASSERT_EQ(sensor_model.beam_mask().size(), 1U);
    EXPECT_TRUE(sensor_model.beam_mask()[0]);
  }
}

}  // namespace
