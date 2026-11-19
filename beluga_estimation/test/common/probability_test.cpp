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

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <beluga_estimation/common/probability.hpp>
#include <beluga_estimation/common/types.hpp>

namespace {

using namespace beluga_estimation;

TEST(IsCovarianceMatrixTest, AcceptsSymmetricPositiveDefinite) {
  beluga_estimation::MatrixXd covariance(2, 2);
  covariance << 2.0, 1.0, 1.0, 3.0;
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
}

TEST(IsCovarianceMatrixTest, AcceptsIdentity) {
  beluga_estimation::MatrixXd covariance = beluga_estimation::MatrixXd::Identity(3, 3);
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
}

TEST(IsCovarianceMatrixTest, AcceptsFixedSizeRowMajorMatrix) {
  beluga_estimation::Matrix3d covariance;
  covariance.setIdentity();
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
}

TEST(IsCovarianceMatrixTest, AcceptsMatrixExpression) {
  // The const reference binds expressions directly and evaluates them into a
  // temporary only when necessary, so callers need not materialize first.
  const auto expression = 2.0 * beluga_estimation::MatrixXd::Identity(3, 3);
  EXPECT_TRUE(IsCovarianceMatrix(expression));
}

TEST(IsCovarianceMatrixTest, WorksWithFloatScalar) {
  beluga_estimation::MatrixXf covariance = beluga_estimation::MatrixXf::Identity(2, 2);
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
}

TEST(IsCovarianceMatrixTest, RejectsAsymmetricMatrix) {
  // The lower triangle is positive definite but the matrix is not symmetric;
  // the symmetry check must reject it before any Cholesky-based use.
  beluga_estimation::MatrixXd covariance(2, 2);
  covariance << 2.0, 1.0, 0.0, 3.0;
  EXPECT_FALSE(IsCovarianceMatrix(covariance));
}

TEST(IsCovarianceMatrixTest, RejectsNonPositiveDefiniteMatrix) {
  beluga_estimation::MatrixXd covariance(2, 2);
  covariance << -1.0, 0.0, 0.0, -1.0;
  EXPECT_FALSE(IsCovarianceMatrix(covariance));
}

TEST(IsCovarianceMatrixTest, RejectsNonSquareMatrix) {
  beluga_estimation::MatrixXd covariance(2, 3);
  covariance << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  EXPECT_FALSE(IsCovarianceMatrix(covariance));
}

}  // namespace
