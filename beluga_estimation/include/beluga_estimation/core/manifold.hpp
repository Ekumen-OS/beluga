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

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include <range/v3/view/single.hpp>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>

namespace beluga_estimation {

/**
 * @brief Interface for manifold operations.
 *
 * Defines the retraction (Plus) and local (Minus) operations on a manifold,
 * along with optional Jacobian and Hessian computations for linearization.
 * Also provides sub-manifold extraction and ambient/tangent transport
 * operations (Lift/Embed) between a manifold and its sub-manifolds.
 */
template <typename T>
class Manifold_ : public std::enable_shared_from_this<Manifold_<T>> {
 public:
  virtual ~Manifold_() = default;

  /// @brief Returns the dimension of the ambient space.
  virtual std::size_t AmbientSize() const = 0;

  /**
   * @brief Returns the ambient size of the sub-manifold defined by the given
   * tangent-space slices.
   *
   * This is a cheap query that avoids constructing the sub-manifold (see
   * @ref GetSubManifold). It is used to size ambient-space buffers before
   * @ref LiftAmbient / @ref EmbedAmbient. The selection rules match those of
   * the ambient operations: unsupported selections (e.g., a partial selection
   * of SE(2)) throw.
   *
   * @param tangent_dims Sorted, non-overlapping slices describing the subset
   *   of tangent dimensions that define the sub-manifold.
   * @return The ambient size of the sub-manifold.
   * @throws std::invalid_argument if @p tangent_dims are not sorted and
   *   non-overlapping, are out of bounds, or define an unsupported sub-manifold.
   */
  virtual std::size_t AmbientSize(beluga_estimation::view<const contiguous_slice> tangent_dims) const = 0;

  /// @overload
  std::size_t AmbientSize(const contiguous_slice& tangent_dims) const {
    return AmbientSize(ranges::views::single(tangent_dims));
  }

  /// @brief Returns the dimension of the tangent space.
  virtual std::size_t TangentSize() const = 0;

  /**
   * @brief Plus($\oplus$) operation (State + Tangent -> State).
   * Corresponds to the "retract" operator or Exp map approximation.
   * Computes result = lhs $\oplus$ rhs.
   *
   * @param lhs State variable ($x$).
   * @param rhs Tangent vector ($\delta$).
   * @param result Output state.
   * @param jacobian Optional: Linearization ($J = \partial(x \oplus \delta) / \partial \cdot$).
   * @param hessian Optional: Curvature information. Used for covariance propagation or 2nd-order methods.
   */
  virtual void Plus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const = 0;

  /**
   * @brief Minus ($\ominus$) operation (State - State -> Tangent).
   * Corresponds to the "retract" operator  or Log map approximation.
   * Computes result = lhs $\ominus$ rhs (i.e. result = $x_2 \ominus x_1$).
   *
   * @param lhs Target state ($x_2$).
   * @param rhs Source state ($x_1$).
   * @param result Output tangent vector.
   * @param jacobian Optional: Linearization ($J = \partial(x_2 \ominus x_1) / \partial \cdot$).
   * @param hessian Optional: Second-order derivatives.
   */
  virtual void Minus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> hessian = std::nullopt) const = 0;

  /**
   * @brief Lifts a sub-manifold tangent vector into the full tangent space.
   *
   * The sub-manifold is characterized by a set of contiguous tangent-space
   * slices of this manifold. The input vector holds the selected tangent
   * dimensions concatenated in the order they appear in @p tangent_dims.
   * Non-selected dimensions of the output are left untouched, allowing
   * callers to accumulate a full state vector across multiple lifts.
   *
   * The default implementation is pure dimension arithmetic and is valid for
   * any manifold, because tangent spaces are always vector spaces. Note that
   * this is more permissive than @ref GetSubManifold: a partial selection of a
   * manifold that does not support proper sub-manifolds (e.g., SE(2)) is still
   * a valid tangent-space operation.
   *
   * @param sub_tangent Input tangent vector of the sub-manifold (size equal to
   *   the total number of selected dimensions).
   * @param tangent_dims Sorted, non-overlapping slices describing the subset
   *   of tangent dimensions that define the sub-manifold.
   * @param full_tangent Output tangent vector of this manifold (size `TangentSize()`).
   * @throws std::invalid_argument if @p tangent_dims are not sorted and
   *   non-overlapping, or are out of bounds.
   */
  virtual void LiftTangent(
      std_future::span<const T> sub_tangent,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> full_tangent) const {
    ValidateTangentDims(tangent_dims, TangentSize());
    BELUGA_ESTIMATION_ASSERT(sub_tangent.size() == SelectedTangentSize(tangent_dims, TangentSize()));
    BELUGA_ESTIMATION_ASSERT(full_tangent.size() == TangentSize());

    std::size_t sub_offset = 0;
    for (const auto& dim : tangent_dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t size = dim.end.value_or(TangentSize()) - start;
      auto dst = full_tangent.subspan(start, size);
      auto src = sub_tangent.subspan(sub_offset, size);
      std::copy(src.begin(), src.end(), dst.begin());
      sub_offset += size;
    }
  }

  /**
   * @brief Embeds (extracts) a sub-manifold tangent vector from the full tangent space.
   *
   * This is the inverse of @ref LiftTangent on the selected dimensions: it
   * copies the selected tangent dimensions into the output, concatenated in
   * the order they appear in @p tangent_dims.
   *
   * @param full_tangent Input tangent vector of this manifold (size `TangentSize()`).
   * @param tangent_dims Sorted, non-overlapping slices describing the subset
   *   of tangent dimensions to extract.
   * @param sub_tangent Output tangent vector of the sub-manifold (size equal to
   *   the total number of selected dimensions).
   * @throws std::invalid_argument if @p tangent_dims are not sorted and
   *   non-overlapping, or are out of bounds.
   */
  virtual void EmbedTangent(
      std_future::span<const T> full_tangent,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> sub_tangent) const {
    ValidateTangentDims(tangent_dims, TangentSize());
    BELUGA_ESTIMATION_ASSERT(full_tangent.size() == TangentSize());
    BELUGA_ESTIMATION_ASSERT(sub_tangent.size() == SelectedTangentSize(tangent_dims, TangentSize()));

    std::size_t sub_offset = 0;
    for (const auto& dim : tangent_dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t size = dim.end.value_or(TangentSize()) - start;
      auto dst = sub_tangent.subspan(sub_offset, size);
      auto src = full_tangent.subspan(start, size);
      std::copy(src.begin(), src.end(), dst.begin());
      sub_offset += size;
    }
  }

  /**
   * @brief Lifts a sub-manifold tangent matrix into the full tangent space.
   *
   * The same selection is applied to the rows and columns of the output
   * matrix (the default column subspace is the row subspace), placing a
   * square sub-block over this manifold's tangent space. This is used to
   * place covariance and Jacobian blocks at the correct positions of the
   * full state. Non-selected entries are left untouched.
   *
   * @param sub_matrix Input matrix with one row and one column per selected
   *   tangent dimension.
   * @param tangent_dims Sorted, non-overlapping slices describing the subset
   *   of tangent dimensions to lift (rows and columns).
   * @param full_matrix Output matrix over this manifold's tangent space.
   * @throws std::invalid_argument if @p tangent_dims are not sorted and
   *   non-overlapping, or are out of bounds.
   */
  virtual void LiftTangent(
      std_future::mdspan<const T, 2> sub_matrix,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::mdspan<T, 2> full_matrix) const {
    LiftTangent(sub_matrix, tangent_dims, tangent_dims, full_matrix);
  }

  /**
   * @brief Lifts a sub-manifold tangent matrix into the full tangent space.
   *
   * The selected rows of the input matrix are copied into the corresponding
   * rows of the output matrix, and its selected columns into the
   * corresponding columns; non-selected entries are left untouched. This is
   * used to place rectangular Jacobian blocks, whose row and column subspaces
   * may be distinct subsets of this manifold's tangent space.
   *
   * @param sub_matrix Input matrix with one row per selected row dimension and
   *   one column per selected column dimension.
   * @param row_dims Sorted, non-overlapping slices describing the subset of
   *   tangent dimensions to lift (rows).
   * @param col_dims Sorted, non-overlapping slices describing the subset of
   *   tangent dimensions to lift (columns).
   * @param full_matrix Output matrix over this manifold's tangent space.
   * @throws std::invalid_argument if @p row_dims or @p col_dims are not sorted
   *   and non-overlapping, or are out of bounds.
   */
  virtual void LiftTangent(
      std_future::mdspan<const T, 2> sub_matrix,
      beluga_estimation::view<const contiguous_slice> row_dims,
      beluga_estimation::view<const contiguous_slice> col_dims,
      std_future::mdspan<T, 2> full_matrix) const {
    ValidateTangentDims(row_dims, TangentSize());
    ValidateTangentDims(col_dims, TangentSize());
    BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(0) == SelectedTangentSize(row_dims, TangentSize()));
    BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(1) == SelectedTangentSize(col_dims, TangentSize()));

    std::size_t sub_row = 0;
    for (const auto& row_dim : row_dims) {
      const std::size_t row_start = row_dim.start.value_or(0);
      const std::size_t row_size = row_dim.end.value_or(TangentSize()) - row_start;
      std::size_t sub_column = 0;
      for (const auto& col_dim : col_dims) {
        const std::size_t col_start = col_dim.start.value_or(0);
        const std::size_t col_size = col_dim.end.value_or(TangentSize()) - col_start;
        auto dst = std_future::submdspan(
            full_matrix, std::pair{row_start, row_start + row_size}, std::pair{col_start, col_start + col_size});
        auto src = std_future::submdspan(
            sub_matrix, std::pair{sub_row, sub_row + row_size}, std::pair{sub_column, sub_column + col_size});
        std_future::copy(dst, src);
        sub_column += col_size;
      }
      sub_row += row_size;
    }
  }

  /**
   * @brief Embeds (extracts) a sub-manifold tangent matrix from the full tangent space.
   *
   * The same selection is applied to the rows and columns of the input matrix
   * (the default column subspace is the row subspace), extracting a square
   * sub-block over this manifold's tangent space.
   *
   * @param full_matrix Input matrix over this manifold's tangent space.
   * @param tangent_dims Sorted, non-overlapping slices describing the subset
   *   of tangent dimensions to extract (rows and columns).
   * @param sub_matrix Output matrix with one row and one column per selected
   *   tangent dimension.
   * @throws std::invalid_argument if @p tangent_dims are not sorted and
   *   non-overlapping, or are out of bounds.
   */
  virtual void EmbedTangent(
      std_future::mdspan<const T, 2> full_matrix,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::mdspan<T, 2> sub_matrix) const {
    EmbedTangent(full_matrix, tangent_dims, tangent_dims, sub_matrix);
  }

  /**
   * @brief Embeds (extracts) a sub-manifold tangent matrix from the full tangent space.
   *
   * The selected rows and columns of the input matrix are copied into the
   * output matrix, in order. This is the inverse of @ref LiftTangent on the
   * selected rows and columns.
   *
   * @param full_matrix Input matrix over this manifold's tangent space.
   * @param row_dims Sorted, non-overlapping slices describing the subset of
   *   tangent dimensions to extract (rows).
   * @param col_dims Sorted, non-overlapping slices describing the subset of
   *   tangent dimensions to extract (columns).
   * @param sub_matrix Output matrix with one row per selected row dimension and
   *   one column per selected column dimension.
   * @throws std::invalid_argument if @p row_dims or @p col_dims are not sorted
   *   and non-overlapping, or are out of bounds.
   */
  virtual void EmbedTangent(
      std_future::mdspan<const T, 2> full_matrix,
      beluga_estimation::view<const contiguous_slice> row_dims,
      beluga_estimation::view<const contiguous_slice> col_dims,
      std_future::mdspan<T, 2> sub_matrix) const {
    ValidateTangentDims(row_dims, TangentSize());
    ValidateTangentDims(col_dims, TangentSize());
    BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(0) == SelectedTangentSize(row_dims, TangentSize()));
    BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(1) == SelectedTangentSize(col_dims, TangentSize()));

    std::size_t sub_row = 0;
    for (const auto& row_dim : row_dims) {
      const std::size_t row_start = row_dim.start.value_or(0);
      const std::size_t row_size = row_dim.end.value_or(TangentSize()) - row_start;
      std::size_t sub_column = 0;
      for (const auto& col_dim : col_dims) {
        const std::size_t col_start = col_dim.start.value_or(0);
        const std::size_t col_size = col_dim.end.value_or(TangentSize()) - col_start;
        auto dst = std_future::submdspan(
            sub_matrix, std::pair{sub_row, sub_row + row_size}, std::pair{sub_column, sub_column + col_size});
        auto src = std_future::submdspan(
            full_matrix, std::pair{row_start, row_start + row_size}, std::pair{col_start, col_start + col_size});
        std_future::copy(dst, src);
        sub_column += col_size;
      }
      sub_row += row_size;
    }
  }

  /**
   * @brief Lifts a sub-manifold ambient vector into the full ambient space.
   *
   * The sub-manifold is characterized by a set of contiguous tangent-space
   * slices of this manifold. The input is copied into the selected
   * sub-manifold block(s), leaving non-selected dimensions untouched.
   *
   * @param sub_ambient Input ambient vector of the sub-manifold.
   * @param tangent_dims Sorted, non-overlapping slices describing the subset
   *   of tangent dimensions that define the sub-manifold.
   * @param full_ambient Output ambient vector of this manifold (size `AmbientSize()`).
   * @throws std::invalid_argument if @p tangent_dims are not sorted and
   *   non-overlapping, are out of bounds, or define an unsupported sub-manifold.
   */
  virtual void LiftAmbient(
      std_future::span<const T> sub_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> full_ambient) const = 0;

  /**
   * @brief Embeds (extracts) a sub-manifold ambient vector from the full ambient space.
   *
   * This is the inverse of @ref LiftAmbient on the selected dimensions: it
   * copies the ambient representation of the selected sub-manifold block(s)
   * into the output, concatenated in the order they appear in @p tangent_dims.
   *
   * @param full_ambient Input ambient vector of this manifold (size `AmbientSize()`).
   * @param tangent_dims Sorted, non-overlapping slices describing the subset
   *   of tangent dimensions that define the sub-manifold.
   * @param sub_ambient Output ambient vector of the sub-manifold.
   * @throws std::invalid_argument if @p tangent_dims are not sorted and
   *   non-overlapping, are out of bounds, or define an unsupported sub-manifold.
   */
  virtual void EmbedAmbient(
      std_future::span<const T> full_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> sub_ambient) const = 0;

  /// @overload
  void LiftTangent(
      std_future::span<const T> sub_tangent,
      const contiguous_slice& tangent_dims,
      std_future::span<T> full_tangent) const {
    return LiftTangent(sub_tangent, ranges::views::single(tangent_dims), full_tangent);
  }

  /// @overload
  void EmbedTangent(
      std_future::span<const T> full_tangent,
      const contiguous_slice& tangent_dims,
      std_future::span<T> sub_tangent) const {
    return EmbedTangent(full_tangent, ranges::views::single(tangent_dims), sub_tangent);
  }

  /// @overload
  void LiftTangent(
      std_future::mdspan<const T, 2> sub_matrix,
      const contiguous_slice& tangent_dims,
      std_future::mdspan<T, 2> full_matrix) const {
    return LiftTangent(sub_matrix, ranges::views::single(tangent_dims), full_matrix);
  }

  /// @overload
  void EmbedTangent(
      std_future::mdspan<const T, 2> full_matrix,
      const contiguous_slice& tangent_dims,
      std_future::mdspan<T, 2> sub_matrix) const {
    return EmbedTangent(full_matrix, ranges::views::single(tangent_dims), sub_matrix);
  }

  /// @overload
  void LiftTangent(
      std_future::mdspan<const T, 2> sub_matrix,
      const contiguous_slice& row_dims,
      const contiguous_slice& col_dims,
      std_future::mdspan<T, 2> full_matrix) const {
    return LiftTangent(sub_matrix, ranges::views::single(row_dims), ranges::views::single(col_dims), full_matrix);
  }

  /// @overload
  void EmbedTangent(
      std_future::mdspan<const T, 2> full_matrix,
      const contiguous_slice& row_dims,
      const contiguous_slice& col_dims,
      std_future::mdspan<T, 2> sub_matrix) const {
    return EmbedTangent(full_matrix, ranges::views::single(row_dims), ranges::views::single(col_dims), sub_matrix);
  }

  /// @overload
  void LiftAmbient(
      std_future::span<const T> sub_ambient,
      const contiguous_slice& tangent_dims,
      std_future::span<T> full_ambient) const {
    return LiftAmbient(sub_ambient, ranges::views::single(tangent_dims), full_ambient);
  }

  /// @overload
  void EmbedAmbient(
      std_future::span<const T> full_ambient,
      const contiguous_slice& tangent_dims,
      std_future::span<T> sub_ambient) const {
    return EmbedAmbient(full_ambient, ranges::views::single(tangent_dims), sub_ambient);
  }

  /**
   * @brief Extracts a sub-manifold.
   *
   * The sub-manifold is characterized by a linear subspace of the tangent space.
   * Each slice in @p tangent_dims selects a set of contiguous dimensions to include
   * in the sub-manifold.
   *
   * @warning The slices in @p tangent_dims must not overlap; each dimension may be
   *   selected at most once. Violations are a precondition error.
   *
   * @param tangent_dims Contiguous slices describing the subset of tangent dimensions
   *   that define the sub-manifold.
   * @return Shared pointer to the sub-manifold, or nullptr if extraction fails.
   */
  [[nodiscard]] virtual std::shared_ptr<const Manifold_<T>> GetSubManifold(const contiguous_slice& tangent_dims) const {
    return GetSubManifold(ranges::views::single(tangent_dims));
  }

  /// @overload
  [[nodiscard]] virtual std::shared_ptr<const Manifold_<T>> GetSubManifold(
      beluga_estimation::view<const contiguous_slice> tangent_dims) const = 0;

 protected:
  /// @brief Validates that tangent dims are sorted, non-overlapping, and in bounds.
  /// @param tangent_dims Slices describing a subset of tangent dimensions.
  /// @param tangent_size The total tangent dimension of the manifold.
  /// @throws std::invalid_argument if @p tangent_dims is empty, or if the
  ///   slices are not sorted and non-overlapping, are inverted, or are out
  ///   of bounds.
  static void ValidateTangentDims(
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std::size_t tangent_size) {
    if (tangent_dims.size() == 0) {
      BELUGA_ESTIMATION_DEMAND(tangent_dims.size() != 0, "tangent dims must not be empty");
    }
    if (!beluga_estimation::disjoint(tangent_dims)) {
      BELUGA_ESTIMATION_DEMAND(
          beluga_estimation::disjoint(tangent_dims), "tangent dims must be sorted and non-overlapping");
    }
    for (const auto& dim : tangent_dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t end = dim.end.value_or(tangent_size);
      BELUGA_ESTIMATION_DEMAND(
          start < end && end <= tangent_size, "tangent dims are inverted, empty, or out of bounds");
    }
  }

  /// @brief Returns the total number of selected tangent dimensions.
  static std::size_t SelectedTangentSize(
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std::size_t tangent_size) {
    std::size_t total = 0;
    for (const auto& dim : tangent_dims) {
      const std::size_t start = dim.start.value_or(0);
      const std::size_t end = dim.end.value_or(tangent_size);
      total += end - start;
    }
    return total;
  }
};

using Manifold = Manifold_<DefaultScalar>;

}  // namespace beluga_estimation
