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

#include <gmock/gmock.h>

#include <beluga/algorithm/thrun_recovery_probability_estimator.hpp>

namespace {

TEST(ThrunRecoveryProbabilityEstimator, ProbabilityWithNoParticles) {
  // Test the probability when the particle set is empty.
  const double alpha_slow = 0.2;
  const double alpha_fast = 0.4;
  auto estimator = beluga::ThrunRecoveryProbabilityEstimator{alpha_slow, alpha_fast};
  estimator.update(std::vector<std::tuple<int, beluga::Weight>>{});
  ASSERT_EQ(estimator(), 0.0);
}

TEST(ThrunRecoveryProbabilityEstimator, ProbabilityWithZeroWeight) {
  // Test the probability when the total weight is zero.
  const double alpha_slow = 0.2;
  const double alpha_fast = 0.4;
  auto estimator = beluga::ThrunRecoveryProbabilityEstimator{alpha_slow, alpha_fast};
  estimator.update(std::vector<std::tuple<int, beluga::Weight>>{{1, 0.0}, {2, 0.0}});
  ASSERT_EQ(estimator(), 0.0);
}

TEST(ThrunRecoveryProbabilityEstimator, ProbabilityAfterUpdateAndReset) {
  const double alpha_slow = 0.5;
  const double alpha_fast = 1.0;
  auto estimator = beluga::ThrunRecoveryProbabilityEstimator{alpha_slow, alpha_fast};

  // Test the probability after updating the estimator with particle weights.
  estimator.update(std::vector<std::tuple<int, beluga::Weight>>{{5, 1.0}, {6, 2.0}, {7, 3.0}});
  ASSERT_EQ(estimator(), 0.0);

  estimator.update(std::vector<std::tuple<int, beluga::Weight>>{{5, 0.5}, {6, 1.0}, {7, 1.5}});
  ASSERT_NEAR(estimator(), 0.33, 0.01);

  estimator.update(std::vector<std::tuple<int, beluga::Weight>>{{5, 0.5}, {6, 1.0}, {7, 1.5}});
  ASSERT_NEAR(estimator(), 0.20, 0.01);

  // Test the probability after resetting the estimator.
  estimator.reset();
  ASSERT_EQ(estimator(), 0.0);
}

class ThrunRecoveryProbabilityWithParam : public ::testing::TestWithParam<std::tuple<double, double, double>> {};

TEST_P(ThrunRecoveryProbabilityWithParam, Probabilities) {
  const auto [initial_weight, final_weight, expected_probability] = GetParam();

  const double alpha_slow = 0.001;
  const double alpha_fast = 0.1;
  auto estimator = beluga::ThrunRecoveryProbabilityEstimator{alpha_slow, alpha_fast};
  auto particles = std::vector<std::tuple<int, beluga::Weight>>{{1, initial_weight}};

  estimator.update(particles);
  ASSERT_NEAR(estimator(), 0.0, 0.01);

  beluga::weight(particles.front()) = final_weight;

  estimator.update(particles);
  ASSERT_NEAR(estimator(), expected_probability, 0.01);
}

INSTANTIATE_TEST_SUITE_P(
    ThrunRecoveryProbability,
    ThrunRecoveryProbabilityWithParam,
    testing::Values(
        std::make_tuple(1.0, 1.5, 0.00),
        std::make_tuple(1.0, 2.0, 0.00),
        std::make_tuple(1.0, 0.5, 0.05),
        std::make_tuple(0.5, 0.1, 0.08),
        std::make_tuple(0.5, 0.0, 0.10)));

}  // namespace
