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

#pragma once

/**
 * @file
 * @brief Includes probability-related utilities.
 */

#include <cassert>

#include <Eigen/Cholesky>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation {

/**
 * @brief Checks whether a matrix is a valid covariance matrix.
 *
 * A valid covariance matrix is square, symmetric, and positive definite.
 * The check is performed in place on the given matrix reference; the matrix
 * is not modified. Being a const reference, it also binds to matrix
 * expressions, which are evaluated into a temporary only when necessary.
 *
 * @param covariance The matrix (or matrix expression) to check.
 * @return `true` if the matrix is a valid covariance matrix, `false` otherwise.
 */
template <typename Derived>
bool IsCovarianceMatrix(const beluga_estimation::MatrixBase<Derived>& covariance) {
  if (covariance.rows() != covariance.cols()) {
    return false;
  }
  // Eigen's LLT only reads the lower triangle, so symmetry must be checked
  // explicitly for an asymmetric matrix to be rejected.
  return covariance.isApprox(covariance.transpose()) && covariance.llt().info() == Eigen::Success;
}

/**
 * @brief Extracts the 3×3 SE(2) covariance `[x, y, yaw]` (or `[vx, vy, wz]`)
 * from a full 6×6 covariance in the SE(3) tangent space.
 *
 * Extracts the 2×2 translation block (position or linear velocity) and the
 * yaw variance from the rotation block (orientation or angular velocity),
 * preserving the cross-covariance terms between translation and rotation.
 *
 * @tparam Derived The Eigen matrix expression type (must be 6×6).
 * @param covariance The full 6×6 covariance, either the pose covariance
 * `[x, y, z, ωx, ωy, ωz]` or the twist covariance `[vx, vy, vz, ωx, ωy, ωz]`.
 * @return The assembled 3×3 SE(2) covariance.
 */
template <typename Derived>
beluga_estimation::Matrix3<typename Derived::Scalar> ToSE2Covariance(
    const beluga_estimation::MatrixBase<Derived>& covariance) {
  static_assert(
      Derived::RowsAtCompileTime == Eigen::Dynamic || Derived::RowsAtCompileTime == 6,
      "ToSE2Covariance requires a 6x6 covariance matrix.");
  static_assert(
      Derived::ColsAtCompileTime == Eigen::Dynamic || Derived::ColsAtCompileTime == 6,
      "ToSE2Covariance requires a 6x6 covariance matrix.");
  BELUGA_ESTIMATION_ASSERT(covariance.rows() == 6 && covariance.cols() == 6);

  using Scalar = typename Derived::Scalar;
  beluga_estimation::Matrix3<Scalar> se2_covariance;
  se2_covariance.template block<2, 2>(0, 0) = covariance.template block<2, 2>(0, 0);
  se2_covariance(2, 2) = covariance(5, 5);
  se2_covariance(0, 2) = covariance(0, 5);
  se2_covariance(1, 2) = covariance(1, 5);
  se2_covariance(2, 0) = covariance(5, 0);
  se2_covariance(2, 1) = covariance(5, 1);
  return se2_covariance;
}

}  // namespace beluga_estimation
