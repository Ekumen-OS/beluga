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

#include <gtest/gtest.h>

#include <sstream>

#include <Eigen/Core>
#include <sophus/common.hpp>
#include <sophus/se2.hpp>

#include "beluga/sensor/data/ndt_cell.hpp"

namespace beluga {

namespace {

Eigen::Matrix2Xd get_diagonal_covariance(double x_var = 0.5, double y_var = 0.8) {
  return Eigen::Vector2d{x_var, y_var}.asDiagonal();
}

}  // namespace

TEST(NdtCellTests, Product) {
  {
    const NdtCell2d cell{Eigen::Vector2d{1, 2}, get_diagonal_covariance()};
    const Sophus::SE2d tf{};
    const auto transformed = tf * cell;
    ASSERT_TRUE(cell.mean.isApprox(transformed.mean));
    ASSERT_TRUE(cell.covariance.isApprox(transformed.covariance));
  }
  {
    const NdtCell2d cell{Eigen::Vector2d{1, 2}, get_diagonal_covariance()};
    const Sophus::SE2d tf{Sophus::SO2d{}, Eigen::Vector2d{1, 2}};
    const auto transformed = tf * cell;
    ASSERT_TRUE(cell.mean.isApprox(transformed.mean - Eigen::Vector2d{1, 2}));
    ASSERT_TRUE(cell.covariance.isApprox(transformed.covariance));
  }
  {
    const NdtCell2d cell{Eigen::Vector2d{1, 2}, get_diagonal_covariance()};
    const Sophus::SE2d tf{Sophus::SO2d{Sophus::Constants<double>::pi() / 2}, Eigen::Vector2d::Zero()};

    Eigen::Matrix<double, 2, 2> expected_transformed_cov;
    // clang-format off
    expected_transformed_cov << 0.8, 0.0,
                                0.0, 0.5;
    // clang-format on
    const auto transformed = tf * cell;
    EXPECT_TRUE(transformed.covariance.isApprox(expected_transformed_cov));
    EXPECT_TRUE(transformed.mean.isApprox(Eigen::Vector2d{-2, 1})) << transformed;
  }
}

TEST(NdtCellTests, Ostream) {
  const NdtCell2d cell{Eigen::Vector2d{1, 2}, get_diagonal_covariance()};
  std::stringstream ss;
  ss << cell;
  ASSERT_GT(ss.str().size(), 0);
}

}  // namespace beluga
