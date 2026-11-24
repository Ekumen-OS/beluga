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
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/experimental/planar_scan_matcher.hpp>
#include <beluga_estimation/synthetic_scan_pair.hpp>

namespace {

constexpr double kTolerance = 1e-9;
using beluga_estimation::experimental::PlanarScanMatchDiagnostics;
using beluga_estimation::experimental::PlanarScanMatcher;
using beluga_estimation::experimental::PlanarScanMatchOutcome;
using beluga_estimation::experimental::PlanarScanMatchResult;
using beluga_estimation::testing::SyntheticScene;

void ExpectPoseNear(const beluga_estimation::SE2d& actual, const beluga_estimation::SE2d& expected) {
  EXPECT_TRUE(actual.translation().isApprox(expected.translation(), kTolerance));
  EXPECT_NEAR(actual.so2().log(), expected.so2().log(), kTolerance);
}

class FakeMatcher : public PlanarScanMatcher {
 public:
  explicit FakeMatcher(PlanarScanMatchOutcome outcome) : outcome_{outcome} {}

  PlanarScanMatchResult Match(
      beluga_estimation::experimental::PlanarScanView<double> reference,
      beluga_estimation::experimental::PlanarScanView<double> target,
      const beluga_estimation::SE2d& reference_T_target) const override {
    if (!AllFinite(reference) || !AllFinite(target)) {
      return {PlanarScanMatchOutcome::kInvalidInput, std::nullopt, {}};
    }
    const bool keep_candidate =
        outcome_ == PlanarScanMatchOutcome::kConverged || outcome_ == PlanarScanMatchOutcome::kRejected ||
        outcome_ == PlanarScanMatchOutcome::kDegenerate || outcome_ == PlanarScanMatchOutcome::kNoConvergence;
    return {
        outcome_, keep_candidate ? std::optional{reference_T_target} : std::nullopt,
        PlanarScanMatchDiagnostics{0.25, 3, static_cast<std::size_t>(reference.size() + target.size())}};
  }

 private:
  static bool AllFinite(beluga_estimation::experimental::PlanarScanView<double> scan) {
    for (const auto& point : scan) {
      if (!point.allFinite()) {
        return false;
      }
    }
    return true;
  }

  PlanarScanMatchOutcome outcome_;
};

TEST(PlanarScanMatcher, AcceptsSpansAndPreservesTransformConvention) {
  beluga_estimation::RandomNumberGenerator generator(42);
  const beluga_estimation::SE2d expected(beluga_estimation::SO2d(0.4), beluga_estimation::Vector2d(1.0, -2.0));
  const auto pair =
      beluga_estimation::testing::MakeSyntheticScanPair<double>(SyntheticScene::kAsymmetricRoom, expected, generator);
  const FakeMatcher matcher(PlanarScanMatchOutcome::kConverged);
  const auto result = matcher.Match(pair.reference, pair.target, expected);
  ASSERT_TRUE(result.reference_T_target);
  ExpectPoseNear(*result.reference_T_target, expected);
  ASSERT_TRUE(result.diagnostics.candidate_count);
  EXPECT_EQ(*result.diagnostics.candidate_count, pair.reference.size() + pair.target.size());
}

TEST(PlanarScanMatcher, ReportsEveryOutcomeWithPermittedEstimateSemantics) {
  constexpr std::array outcomes{
      PlanarScanMatchOutcome::kConverged,    PlanarScanMatchOutcome::kRejected,
      PlanarScanMatchOutcome::kDegenerate,   PlanarScanMatchOutcome::kNoConvergence,
      PlanarScanMatchOutcome::kInvalidInput, PlanarScanMatchOutcome::kNumericalFailure,
  };
  const std::vector<beluga_estimation::Vector2d> scan{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
  for (const auto outcome : outcomes) {
    const auto result = FakeMatcher(outcome).Match(scan, scan, beluga_estimation::SE2d{});
    EXPECT_EQ(result.outcome, outcome);
    const bool candidate_permitted =
        outcome == PlanarScanMatchOutcome::kConverged || outcome == PlanarScanMatchOutcome::kRejected ||
        outcome == PlanarScanMatchOutcome::kDegenerate || outcome == PlanarScanMatchOutcome::kNoConvergence;
    EXPECT_EQ(result.reference_T_target.has_value(), candidate_permitted);
    EXPECT_EQ(result.diagnostics.iteration_count, 3U);
    ASSERT_TRUE(result.diagnostics.candidate_count);
    ASSERT_TRUE(result.diagnostics.score);
    EXPECT_DOUBLE_EQ(*result.diagnostics.score, 0.25);
  }

  auto invalid_scan = scan;
  invalid_scan.front().x() = std::numeric_limits<double>::quiet_NaN();
  const auto invalid =
      FakeMatcher(PlanarScanMatchOutcome::kConverged).Match(scan, invalid_scan, beluga_estimation::SE2d{});
  EXPECT_EQ(invalid.outcome, PlanarScanMatchOutcome::kInvalidInput);
  EXPECT_FALSE(invalid.reference_T_target);
}

}  // namespace
