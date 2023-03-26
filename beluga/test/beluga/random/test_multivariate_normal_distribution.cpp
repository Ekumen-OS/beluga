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

#include <beluga/algorithm/estimation.hpp>
#include <beluga/random/multivariate_normal_distribution.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include "beluga/test/utils/sophus_matchers.hpp"

namespace {

TEST(MultivariateNormalDistribution, CopyAndCompare) {
  Eigen::Matrix3d covariance = Eigen::Vector3d::Ones().asDiagonal();
  auto distribution = beluga::MultivariateNormalDistribution{covariance};
  auto other_distribution = distribution;
  ASSERT_EQ(distribution, other_distribution);
  auto generator = std::mt19937{std::random_device()()};
  auto other_generator = generator;
  // Testing twice to assert that they generate the same sequence.
  ASSERT_EQ(distribution(generator), other_distribution(other_generator));
  ASSERT_EQ(distribution(generator), other_distribution(other_generator));
}

TEST(MultivariateNormalDistribution, NegativeEigenvaluesMatrix) {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Ones();
  EXPECT_THROW(beluga::MultivariateNormalDistribution{covariance}, std::runtime_error);
}

TEST(MultivariateNormalDistribution, NonSymmetricMatrix) {
  Eigen::Matrix3d covariance;
  covariance << 1.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  EXPECT_THROW(beluga::MultivariateNormalDistribution{covariance}, std::runtime_error);
}

TEST(MultivariateNormalDistribution, SampleZero) {
  auto generator = std::mt19937{std::random_device()()};
  auto distribution = beluga::MultivariateNormalDistribution{Eigen::Vector2d::Zero(), Eigen::Matrix2d::Zero()};
  ASSERT_THAT(distribution(generator), testing::Vector2Eq(Eigen::Vector2d{0, 0}));
}

class MultivariateNormalDistributionWithParam
    : public ::testing::TestWithParam<std::pair<Eigen::Vector2d, Eigen::Matrix2d>> {};

TEST_P(MultivariateNormalDistributionWithParam, DistributionCovarianceAndMean) {
  const Eigen::Vector2d expected_mean = std::get<0>(GetParam());
  const Eigen::Matrix2d expected_covariance = std::get<1>(GetParam());
  auto distribution = beluga::MultivariateNormalDistribution{expected_mean, expected_covariance};
  const auto sequence = ranges::views::generate([&]() {
                          static auto generator = std::mt19937{std::random_device()()};
                          return distribution(generator);
                        }) |
                        ranges::views::take_exactly(1'000'000) | ranges::to<std::vector>;

  const auto sum = std::accumulate(sequence.begin(), sequence.end(), Eigen::Vector2d{0, 0});
  const auto mean = Eigen::Vector2d{sum / sequence.size()};
  ASSERT_NEAR(mean(0), expected_mean(0), 0.01);
  ASSERT_NEAR(mean(1), expected_mean(1), 0.01);
  const auto covariance = beluga::calculate_covariance(sequence, mean);
  ASSERT_NEAR(covariance(0, 0), expected_covariance(0, 0), 0.01);
  ASSERT_NEAR(covariance(0, 1), expected_covariance(0, 1), 0.01);
  ASSERT_NEAR(covariance(1, 0), expected_covariance(1, 0), 0.01);
  ASSERT_NEAR(covariance(1, 1), expected_covariance(1, 1), 0.01);
}

INSTANTIATE_TEST_SUITE_P(
    MeanCovariancePairs,
    MultivariateNormalDistributionWithParam,
    testing::Values(
        std::make_pair(Eigen::Vector2d{1.0, 2.0}, Eigen::Matrix2d{{0.0, 0.0}, {0.0, 0.0}}),
        std::make_pair(Eigen::Vector2d{3.0, 4.0}, Eigen::Matrix2d{{1.0, 0.0}, {0.0, 1.0}}),
        std::make_pair(Eigen::Vector2d{3.0, 4.0}, Eigen::Matrix2d{{1.5, -0.3}, {-0.3, 1.5}}),
        std::make_pair(Eigen::Vector2d{5.0, 6.0}, Eigen::Matrix2d{{2.0, 0.7}, {0.7, 2.0}})));

}  // namespace
