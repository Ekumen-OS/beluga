// Copyright 2022 Ekumen, Inc.
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

#include <beluga/random/multivariate_normal_distribution.hpp>

#include "../../utils/sophus_matchers.hpp"

namespace {

TEST(MultivariateNormalDistribution, CopyAndCompare) {
  Eigen::Matrix3d covariance = Eigen::Vector3d::Ones().asDiagonal();
  auto distribution = beluga::MultivariateNormalDistribution{covariance};
  auto other_distribution = distribution;
  ASSERT_EQ(distribution, other_distribution);
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

}  // namespace
