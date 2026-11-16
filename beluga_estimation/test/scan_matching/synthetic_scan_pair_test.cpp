// Copyright 2026 Ekumen, Inc.
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

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <gtest/gtest.h>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/synthetic_scan_pair.hpp>

namespace {

constexpr double kTolerance = 1e-9;
using beluga_estimation::testing::SyntheticScanCorruption;
using beluga_estimation::testing::SyntheticScene;

TEST(SyntheticScanPair, ExactIdentity) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const auto scan_pair = beluga_estimation::testing::MakeSyntheticScanPair<double>(
      SyntheticScene::kAsymmetricRoom, beluga_estimation::SE2d{}, generator);
  ASSERT_EQ(scan_pair.reference.size(), scan_pair.target.size());
  for (std::size_t index = 0; index < scan_pair.reference.size(); ++index) {
    EXPECT_TRUE(scan_pair.reference[index].isApprox(scan_pair.target[index], kTolerance));
  }
}

TEST(SyntheticScanPair, ExactCombinedTransformAndWraparound) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const beluga_estimation::SE2d transform(beluga_estimation::SO2d(3.5), beluga_estimation::Vector2d(1.25, -0.75));
  const auto scan_pair = beluga_estimation::testing::MakeSyntheticScanPair<double>(
      SyntheticScene::kSparseNonCollinear, transform, generator);
  EXPECT_NEAR(scan_pair.reference_T_target.so2().log(), 3.5 - 2.0 * M_PI, kTolerance);
  for (std::size_t index = 0; index < scan_pair.reference.size(); ++index) {
    EXPECT_TRUE((transform * scan_pair.target[index]).isApprox(scan_pair.reference[index], kTolerance));
  }
}

TEST(SyntheticScanPair, IsDeterministicForFixedSeed) {
  SyntheticScanCorruption corruption{0.65, 0.1, 2, 2, 7.0};
  beluga_estimation::RandomNumberGenerator first_generator(123);
  beluga_estimation::RandomNumberGenerator second_generator(123);
  const auto first = beluga_estimation::testing::MakeSyntheticScanPair<double>(
      SyntheticScene::kCorridor, beluga_estimation::SE2d{}, first_generator, corruption);
  const auto second = beluga_estimation::testing::MakeSyntheticScanPair<double>(
      SyntheticScene::kCorridor, beluga_estimation::SE2d{}, second_generator, corruption);
  ASSERT_EQ(first.target.size(), second.target.size());
  for (std::size_t index = 0; index < first.target.size(); ++index) {
    EXPECT_EQ(first.target[index], second.target[index]);
  }
}

TEST(SyntheticScanPair, NoisePreservesTargetBeamDirectionsAndPositiveRanges) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const beluga_estimation::SE2d transform(beluga_estimation::SO2d(0.4), beluga_estimation::Vector2d(1.0, -2.0));
  // Large noise exercises rejection of negative ranges rather than clamping.
  const SyntheticScanCorruption corruption{1.0, 100.0};
  const auto scan_pair = beluga_estimation::testing::MakeSyntheticScanPair<double>(
      SyntheticScene::kRepeatedGeometry, transform, generator, corruption);
  ASSERT_EQ(scan_pair.target.size(), scan_pair.reference.size());
  bool any_changed = false;
  for (std::size_t index = 0; index < scan_pair.target.size(); ++index) {
    const auto expected = transform.inverse() * scan_pair.reference[index];
    EXPECT_GT(scan_pair.target[index].norm(), 0.0);
    EXPECT_TRUE(scan_pair.target[index].allFinite());
    EXPECT_TRUE(scan_pair.target[index].normalized().isApprox(expected.normalized(), kTolerance));
    any_changed = any_changed || !scan_pair.target[index].isApprox(expected, kTolerance);
  }
  EXPECT_TRUE(any_changed);
}

TEST(SyntheticScanPair, ShortReturnsReplaceDistinctMeasurements) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const beluga_estimation::SE2d transform(beluga_estimation::SO2d(0.4), beluga_estimation::Vector2d(1.0, -2.0));
  const auto scan_pair = beluga_estimation::testing::MakeSyntheticScanPair<double>(
      SyntheticScene::kRepeatedGeometry, transform, generator, {1.0, 0.0, 3});
  ASSERT_EQ(scan_pair.target.size(), scan_pair.reference.size());
  std::size_t changed = 0;
  for (std::size_t index = 0; index < scan_pair.target.size(); ++index) {
    const auto expected = transform.inverse() * scan_pair.reference[index];
    if (!scan_pair.target[index].isApprox(expected, kTolerance)) {
      ++changed;
      EXPECT_LT(scan_pair.target[index].norm(), expected.norm());
      EXPECT_TRUE(scan_pair.target[index].normalized().isApprox(expected.normalized(), kTolerance));
    }
  }
  EXPECT_EQ(changed, 3U);
}

TEST(SyntheticScanPair, RandomReturnsCanBeShorterOrLonger) {
  beluga_estimation::RandomNumberGenerator generator(42);
  bool any_shorter = false;
  bool any_longer = false;
  for (int trial = 0; trial < 20; ++trial) {
    const auto scan_pair = beluga_estimation::testing::MakeSyntheticScanPair<double>(
        SyntheticScene::kRepeatedGeometry, beluga_estimation::SE2d{}, generator, {1.0, 0.0, 0, 8, 10.0});
    ASSERT_EQ(scan_pair.target.size(), scan_pair.reference.size());
    for (std::size_t index = 0; index < scan_pair.target.size(); ++index) {
      const auto range = scan_pair.target[index].norm();
      EXPECT_GE(range, 0.0);
      EXPECT_LT(range, 10.0);
      EXPECT_TRUE(scan_pair.target[index].normalized().isApprox(scan_pair.reference[index].normalized(), kTolerance));
      any_shorter = any_shorter || range < scan_pair.reference[index].norm();
      any_longer = any_longer || range > scan_pair.reference[index].norm();
    }
  }
  EXPECT_TRUE(any_shorter);
  EXPECT_TRUE(any_longer);
}

TEST(SyntheticScanPair, RejectsUnsatisfiableReplacementCounts) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const auto make = [&](const SyntheticScanCorruption& corruption) {
    return beluga_estimation::testing::MakeSyntheticScanPair<double>(
        SyntheticScene::kSparseNonCollinear, beluga_estimation::SE2d{}, generator, corruption);
  };
  EXPECT_THROW(make({0.0, 0.0, 1, 0}), std::invalid_argument);
  EXPECT_THROW(make({0.0, 0.0, 0, 1}), std::invalid_argument);
  EXPECT_THROW(make({1.0, 0.0, 4, 0}), std::invalid_argument);
  EXPECT_THROW(make({1.0, 0.0, 0, 4}), std::invalid_argument);
  EXPECT_THROW(make({1.0, 0.0, 2, 2}), std::invalid_argument);
  EXPECT_THROW(make({1.0, 0.0, 1, std::numeric_limits<std::size_t>::max()}), std::invalid_argument);
  EXPECT_TRUE(make({0.0, 0.0}).target.empty());
  EXPECT_NO_THROW(make({1.0, 0.0, 1, 2}));
}

TEST(SyntheticScanPair, MixedReplacementsUseDifferentBeams) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const auto scan_pair = beluga_estimation::testing::MakeSyntheticScanPair<double>(
      SyntheticScene::kRepeatedGeometry, beluga_estimation::SE2d{}, generator, {1.0, 0.0, 2, 3, 10.0});
  ASSERT_EQ(scan_pair.target.size(), scan_pair.reference.size());
  std::size_t changed = 0;
  for (std::size_t index = 0; index < scan_pair.target.size(); ++index) {
    changed += !scan_pair.target[index].isApprox(scan_pair.reference[index], kTolerance);
  }
  EXPECT_EQ(changed, 5U);
}

TEST(SyntheticScanPair, ZeroRangeRemainsUnchanged) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const beluga_estimation::SE2d transform(beluga_estimation::SO2d{}, beluga_estimation::Vector2d(-2.0, -1.0));
  for (const auto corruption : {SyntheticScanCorruption{1.0, 100.0}, SyntheticScanCorruption{1.0, 0.0, 1, 1}}) {
    const auto scan_pair = beluga_estimation::testing::MakeSyntheticScanPair<double>(
        SyntheticScene::kSparseNonCollinear, transform, generator, corruption);
    ASSERT_EQ(scan_pair.target.size(), 3U);
    EXPECT_TRUE(scan_pair.target.front().isZero());
    for (const auto& point : scan_pair.target) {
      EXPECT_TRUE(point.allFinite());
    }
  }
}

TEST(SyntheticScanPair, ReplacementCountsExcludeZeroRangePoints) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const beluga_estimation::SE2d transform(beluga_estimation::SO2d{}, beluga_estimation::Vector2d(-2.0, -1.0));
  EXPECT_THROW(
      beluga_estimation::testing::MakeSyntheticScanPair<double>(
          SyntheticScene::kSparseNonCollinear, transform, generator, {1.0, 0.0, 1, 2}),
      std::invalid_argument);
}

TEST(SyntheticScanPair, ProvidesAllSceneTypes) {
  constexpr std::array scenes{
      SyntheticScene::kAsymmetricRoom, SyntheticScene::kCorridor, SyntheticScene::kRepeatedGeometry,
      SyntheticScene::kSparseNonCollinear};
  for (const auto scene : scenes) {
    const auto points = beluga_estimation::testing::MakeSyntheticScene<double>(scene);
    EXPECT_GE(points.size(), 3U);
    for (const auto& point : points) {
      EXPECT_TRUE(point.allFinite());
    }
  }
}

TEST(SyntheticScanPair, RejectsInvalidConfigurationAndTransform) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const auto make = [&](SyntheticScanCorruption corruption) {
    return beluga_estimation::testing::MakeSyntheticScanPair<double>(
        SyntheticScene::kCorridor, beluga_estimation::SE2d{}, generator, corruption);
  };
  EXPECT_THROW(make({-0.1, 0.0, 0, 0, 1.0}), std::invalid_argument);
  EXPECT_THROW(make({1.1, 0.0, 0, 0, 1.0}), std::invalid_argument);
  EXPECT_THROW(make({1.0, -0.1, 0, 0, 1.0}), std::invalid_argument);
  EXPECT_THROW(make({1.0, 0.0, 0, 0, 0.0}), std::invalid_argument);
  EXPECT_THROW(make({std::numeric_limits<double>::quiet_NaN(), 0.0, 0, 0, 1.0}), std::invalid_argument);

  beluga_estimation::SE2d invalid;
  invalid.translation().x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      beluga_estimation::testing::MakeSyntheticScanPair<double>(SyntheticScene::kCorridor, invalid, generator),
      std::invalid_argument);
}

}  // namespace
