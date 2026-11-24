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
 * @brief Interpolation utilities for algebraic types.
 */

#include <Eigen/Eigenvalues>

#include <beluga_estimation/common/algebra.hpp>

namespace beluga_estimation {

/**
 * @brief Geodesic SE(2) interpolation: `pose_a * exp(alpha * log(pose_a⁻¹ * pose_b))`.
 * @param pose_a Start pose (alpha = 0).
 * @param pose_b End pose (alpha = 1).
 * @param alpha Interpolation parameter in [0, 1].
 */
template <typename T>
SE2<T> InterpolateSE2Pose(const SE2<T>& pose_a, const SE2<T>& pose_b, T alpha) {
  return pose_a * SE2<T>::exp(alpha * (pose_a.inverse() * pose_b).log());
}

// Log-Euclidean interpolation on the SPD manifold: exp((1-α)*log(A) + α*log(B)).
// Avoids the artificial uncertainty inflation produced by linear interpolation.
template <typename T>
Matrix3<T> InterpolateSPD(const Matrix3<T>& cov_a, const Matrix3<T>& cov_b, T alpha) {
  const auto logm = [](const Matrix3<T>& C) {
    Eigen::SelfAdjointEigenSolver<Matrix3<T>> es(C);
    return (es.eigenvectors() * es.eigenvalues().array().log().matrix().asDiagonal() * es.eigenvectors().transpose())
        .eval();
  };
  const auto expm = [](const Matrix3<T>& M) {
    Eigen::SelfAdjointEigenSolver<Matrix3<T>> es(M);
    return (es.eigenvectors() * es.eigenvalues().array().exp().matrix().asDiagonal() * es.eigenvectors().transpose())
        .eval();
  };
  return expm((1 - alpha) * logm(cov_a) + alpha * logm(cov_b));
}

}  // namespace beluga_estimation
