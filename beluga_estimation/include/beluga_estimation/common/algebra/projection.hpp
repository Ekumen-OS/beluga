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
 * @brief Utilities that project a richer algebraic type down to a lower-dimensional one.
 *
 * Coordinate projections are described by boolean masks, Beluga's configuration
 * syntax for selecting components. Selected coordinates preserve their relative
 * order, so non-contiguous selections are supported. Callers either project the
 * coordinates directly with @ref Project, or compile the projection into a
 * matrix once with @ref MakeProjectionMatrix and multiply by it from then on.
 */

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/bitset.hpp>

namespace beluga_estimation {

/// @brief Extracts yaw from a 3D orientation quaternion, projecting SO(3) onto SO(2).
template <typename Derived>
inline double ExtractYaw(const Eigen::QuaternionBase<Derived>& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/**
 * @brief Projects a vector onto the coordinates whose mask entry is set.
 *
 * @tparam Derived Eigen expression type of the source vector.
 * @param input The source vector.
 * @param mask One entry per coordinate of @p input.
 * @return The projected coordinates, in increasing coordinate order.
 */
template <typename Derived, std::enable_if_t<Derived::ColsAtCompileTime == 1, int> = 0>
beluga_estimation::VectorX<typename Derived::Scalar> Project(
    const beluga_estimation::MatrixBase<Derived>& input,
    const std::vector<bool>& mask) {
  BELUGA_ESTIMATION_DEMAND(
      mask.size() == static_cast<std::size_t>(input.size()), "mask length {} does not match the vector size {}",
      mask.size(), input.size());
  const auto indices = IndexBitSet(mask);
  beluga_estimation::VectorX<typename Derived::Scalar> output(static_cast<Eigen::Index>(indices.size()));
  for (Eigen::Index i = 0; i < output.size(); ++i) {
    output(i) = input(static_cast<Eigen::Index>(indices[static_cast<std::size_t>(i)]));
  }
  return output;
}

/**
 * @brief Projects a square matrix onto the rows and columns whose mask entry is set.
 *
 * Useful to project a covariance matrix onto the components being fused.
 *
 * @tparam Derived Eigen expression type of the source matrix.
 * @param input The source square matrix.
 * @param mask One entry per row (and column) of @p input.
 * @return The projected rows and columns, in increasing order.
 */
template <typename Derived, std::enable_if_t<Derived::ColsAtCompileTime != 1, int> = 0>
beluga_estimation::MatrixX<typename Derived::Scalar> Project(
    const beluga_estimation::MatrixBase<Derived>& input,
    const std::vector<bool>& mask) {
  BELUGA_ESTIMATION_DEMAND(input.rows() == input.cols(), "masked projection requires a square matrix");
  BELUGA_ESTIMATION_DEMAND(
      mask.size() == static_cast<std::size_t>(input.rows()), "mask length {} does not match the matrix size {}",
      mask.size(), input.rows());
  const auto indices = IndexBitSet(mask);
  const auto size = static_cast<Eigen::Index>(indices.size());
  beluga_estimation::MatrixX<typename Derived::Scalar> output(size, size);
  for (Eigen::Index i = 0; i < size; ++i) {
    for (Eigen::Index j = 0; j < size; ++j) {
      output(i, j) = input(
          static_cast<Eigen::Index>(indices[static_cast<std::size_t>(i)]),
          static_cast<Eigen::Index>(indices[static_cast<std::size_t>(j)]));
    }
  }
  return output;
}

/**
 * @brief Builds the projection matrix of a boolean mask.
 *
 * The result is a k x n matrix, with k the number of set bits and n the mask
 * length, whose i-th row has a single one at the column of the i-th set bit, in
 * increasing order. Multiplying it by a vector projects the selected
 * coordinates; multiplying its transpose by a k vector scatters them back.
 *
 * Prefer multiplying by it over projecting coordinates one at a time when the
 * projection feeds a linear model, so that the model and the projection compose
 * as matrices.
 *
 * @tparam T The scalar type of the matrix.
 * @param mask One entry per coordinate of the vector it multiplies.
 * @return The k x n projection matrix.
 */
template <typename T>
beluga_estimation::MatrixX<T> MakeProjectionMatrix(const std::vector<bool>& mask) {
  const auto indices = IndexBitSet(mask);
  BELUGA_ESTIMATION_DEMAND(!indices.empty(), "mask must select at least one coordinate");
  beluga_estimation::MatrixX<T> output(
      static_cast<Eigen::Index>(indices.size()), static_cast<Eigen::Index>(mask.size()));
  output.setZero();
  for (Eigen::Index row = 0; row < output.rows(); ++row) {
    output(row, static_cast<Eigen::Index>(indices[static_cast<std::size_t>(row)])) = T{1};
  }
  return output;
}

}  // namespace beluga_estimation
