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
#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>
#include <range/v3/view/zip.hpp>
#include <sophus/common.hpp>
#include <sophus/so2.hpp>

#include "beluga/algorithm/unscented_transform.hpp"

struct UnscentedTransformTests : public ::testing::Test {};

TEST_F(UnscentedTransformTests, Identity) {
  constexpr double kTolerance = 1e-4;
  const Eigen::Vector2d mean{12.3, 7.6};
  Eigen::Matrix<double, 2, 2> cov;
  cov << 1.44, 0.,  //
      0., 2.89;

  const auto [transformed_mean, transformed_cov] =
      beluga::unscented_transform(mean, cov, [](const Eigen::Vector2d& in) -> Eigen::Vector2d { return in; });

  ASSERT_TRUE(transformed_mean.isApprox(mean, kTolerance)) << mean.transpose();
  ASSERT_TRUE(transformed_cov.isApprox(cov, kTolerance)) << cov.transpose();
}

TEST_F(UnscentedTransformTests, Scaling) {
  constexpr double kTolerance = 1e-4;
  const Eigen::Vector2d mean{12.3, 7.6};

  const double k_stdev_x = 1.2;
  const double k_stdev_y = 1.7;
  Eigen::Matrix<double, 2, 2> cov;
  cov << k_stdev_x * k_stdev_x, 0.,  //
      0., k_stdev_y * k_stdev_y;

  const auto [transformed_mean, transformed_cov] =
      beluga::unscented_transform(mean, cov, [](const Eigen::Vector2d& in) -> Eigen::Vector2d { return 2 * in; });

  Eigen::Matrix<double, 2, 2> expected_cov;
  expected_cov << (2. * k_stdev_x) * (2. * k_stdev_x), 0.,  //
      0., (2. * k_stdev_y) * (2. * k_stdev_y);

  ASSERT_TRUE(transformed_mean.isApprox(2. * mean, kTolerance)) << mean.transpose();
  ASSERT_TRUE(transformed_cov.isApprox(expected_cov, kTolerance)) << transformed_cov << " \n" << expected_cov;
}

double wrap_negpi_pi(double a) {
  static const double kPi = Sophus::Constants<double>::pi();
  while (a < kPi) {
    a += 2 * kPi;
  }
  while (a > kPi) {
    a -= 2 * kPi;
  }
  return a;
};

TEST_F(UnscentedTransformTests, CartesianToPolar) {
  // This example is taken from wikipedia, see https://en.wikipedia.org/wiki/Unscented_transform .
  constexpr double kTolerance = 1e-2;
  const Eigen::Vector2d mean{12.3, 7.6};

  Eigen::Matrix<double, 2, 2> cov;
  cov << 1.44, 0.,  //
      0., 2.89;

  const auto [transformed_mean, transformed_cov] =
      beluga::unscented_transform(mean, cov, [](const Eigen::Vector2d& in) -> Eigen::Vector2d {
        return Eigen::Vector2d{std::hypot(in.x(), in.y()), std::atan2(in.y(), in.x())};
      });

  Eigen::Matrix<double, 2, 2> expected_cov;
  expected_cov << 1.823, 0.043,  //
      0.043, 0.012;

  const Eigen::Vector2d k_expected_mean{14.454, 0.55};
  ASSERT_TRUE(transformed_mean.isApprox(k_expected_mean, kTolerance)) << transformed_mean.transpose();

  ASSERT_TRUE(transformed_cov.isApprox(expected_cov, kTolerance)) << transformed_cov.transpose();
}

TEST_F(UnscentedTransformTests, InAndOutDifferentDims) {
  constexpr double kTolerance = 1e-4;
  const Eigen::Vector2d mean{12.3, 7.6};

  Eigen::Matrix<double, 2, 2> cov;
  cov << 1.44, 0.,  //
      0., 2.89;

  const auto [transformed_mean, transformed_cov] =
      beluga::unscented_transform(mean, cov, [](const Eigen::Vector2d& in) -> Eigen::Vector3d {
        return Eigen::Vector3d{in.x(), in.y(), 0.};
      });

  Eigen::Matrix<double, 3, 3> expected_cov;
  expected_cov <<  //
      1.44,
      0., 0.0,        //
      0., 2.89, 0.0,  //
      0., 0., 0.;

  ASSERT_TRUE(transformed_mean.head<2>().isApprox(mean, kTolerance)) << transformed_mean.transpose();

  ASSERT_TRUE(transformed_cov.isApprox(expected_cov, kTolerance)) << transformed_cov.transpose();
}

TEST_F(UnscentedTransformTests, WorksWithEigenExpressions) {
  constexpr double kTolerance = 1e-4;

  const auto [transformed_mean, transformed_cov] = beluga::unscented_transform(
      Eigen::Vector2d::Ones(), Eigen::Matrix<double, 2, 2>::Identity(),
      [](const Eigen::Vector2d& in) -> Eigen::Vector2d { return in; });

  ASSERT_TRUE(transformed_mean.isApprox(Eigen::Vector2d::Ones(), kTolerance));
  ASSERT_TRUE(transformed_cov.isApprox(Eigen::Matrix<double, 2, 2>::Identity()));
}

double angular_distance(double angle1, double angle2) {
  Sophus::SO2<double> rot1(angle1);
  Sophus::SO2<double> rot2(angle2);

  Sophus::SO2<double> relative_rotation = rot1.inverse() * rot2;

  double distance = relative_rotation.log();

  // Ensure the distance is the minimal distance
  return std::min(std::abs(distance), 2 * M_PI - std::abs(distance));
}

TEST_F(UnscentedTransformTests, DifferentMeanFn) {
  constexpr double kTolerance = 1e-1;
  // This mean in polar coordinates is {1.4, pi}. This is specifically chosen so that we're close to the wrap around
  // point.
  const Eigen::Vector2d mean{-1.4, 0.};
  const Eigen::Vector2d expected_mean = {1.4, Sophus::Constants<double>::pi()};

  Eigen::Matrix<double, 2, 2> cov;
  cov << 1e-1, 0.,  //
      0., 1e-1;

  // Demonstrate why using default mean and residual functions can yield bad results, even bad means.
  {
    const auto [transformed_mean, transformed_cov] =
        beluga::unscented_transform(mean, cov, [](const Eigen::Vector2d& in) -> Eigen::Vector2d {
          return Eigen::Vector2d{std::hypot(in.x(), in.y()), std::atan2(in.y(), in.x())};
        });

    const Eigen::Vector2d expected_wrong_mean = {1.4348469228349534, 1.5707963267948966};

    ASSERT_FALSE(transformed_mean.isApprox(expected_mean, kTolerance)) << transformed_mean.transpose();
    ASSERT_TRUE(transformed_mean.isApprox(expected_wrong_mean, kTolerance)) << transformed_mean.transpose();
  }
  {
    const auto [transformed_mean, transformed_cov] = beluga::unscented_transform(
        mean, cov,
        [](const Eigen::Vector2d& in) -> Eigen::Vector2d {
          return Eigen::Vector2d{std::hypot(in.x(), in.y()), std::atan2(in.y(), in.x())};
        },
        std::nullopt,
        [](const std::vector<Eigen::Vector2d>& samples, const std::vector<double>& weights) -> Eigen::Vector2d {
          Eigen::Vector3d aux = Eigen::Vector3d::Zero();
          for (const auto& [s, w] : ranges::views::zip(samples, weights)) {
            aux.x() += s.x() * w;
            aux.y() += std::sin(s.y()) * w;
            aux.z() += std::cos(s.y()) * w;
          }
          return Eigen::Vector2d{aux.x(), std::atan2(aux.y(), aux.z())};
        },
        [](const Eigen::Vector2d& sample, const Eigen::Vector2d& mean) {
          Eigen::Vector2d error;
          error.x() = sample.x() - mean.x();
          error.y() = angular_distance(sample.y(), mean.y());
          return error;
        });
    ASSERT_TRUE(transformed_mean.isApprox(expected_mean, kTolerance)) << transformed_mean.transpose();
  }
}

TEST_F(UnscentedTransformTests, DifferentKappa) {
  constexpr double kTolerance = 1e-4;
  const auto [transformed_mean, transformed_cov] = beluga::unscented_transform(
      Eigen::Vector2d::Ones(), Eigen::Matrix<double, 2, 2>::Identity(),
      [](const Eigen::Vector2d& in) -> Eigen::Vector2d { return in; }, 1 / 3.);

  ASSERT_TRUE(transformed_mean.isApprox(Eigen::Vector2d::Ones(), kTolerance));
  ASSERT_TRUE(transformed_cov.isApprox(Eigen::Matrix<double, 2, 2>::Identity()));
}

TEST_F(UnscentedTransformTests, NegativeKappa) {
  ASSERT_DEBUG_DEATH(beluga::unscented_transform(
                         Eigen::Vector2d::Ones(), Eigen::Matrix<double, 2, 2>::Identity(),
                         [](const Eigen::Vector2d& in) -> Eigen::Vector2d { return in; }, -1 / 3.);
                     , "Assertion");
}
