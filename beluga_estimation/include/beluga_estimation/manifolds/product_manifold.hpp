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
#include <utility>
#include <vector>

#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/manifold.hpp>

namespace beluga_estimation {

/// @brief Axis selector for @ref ProductManifold_::LiftTangent matrix lifts.
///
/// A plain integer rather than an enum so the convention extends to tensor
/// quantities when needed: 0 selects rows, 1 selects columns (higher values
/// would select further tensor axes).
inline constexpr std::size_t kRowsAxis = 0;
inline constexpr std::size_t kColsAxis = 1;

template <typename T>
class ProductManifoldBuilder_;

/// @brief Product manifold formed by one or more sub-manifolds.
///
/// Represents a manifold that is the Cartesian product of a sequence of
/// sub-manifolds (e.g., $SE(2) \times \mathbb{R}^3$). The product manifold
/// supports the full manifold interface: `Plus`, `Minus`, `GetSubManifold`,
/// and the ambient/tangent `Lift`/`Embed` operations.
///
/// ### Run-length compaction
///
/// Consecutive sub-manifolds with the same type are automatically merged
/// into a single block during construction, reducing storage and loop
/// overhead. For example, a sequence of five identical 1D Euclidean
/// manifolds is stored as a single block with `size = 5`.
///
/// @tparam T The numeric scalar type (e.g., `double`, `float`).
template <typename T>
class ProductManifold_ : public Manifold_<T> {
 public:
  /// @brief Constructs a product manifold from a sequence of sub-manifolds.
  ///
  /// The constructor delegates to @ref ProductManifoldBuilder_, which
  /// performs run-length compaction on consecutive identical manifolds.
  ///
  /// @param sub_manifolds A view over the ordered sequence of sub-manifolds
  ///   that make up the product.
  explicit ProductManifold_(beluga_estimation::view<std::shared_ptr<const Manifold_<T>>> sub_manifolds) {
    ProductManifoldBuilder_<T> builder;
    for (auto& m : sub_manifolds) {
      builder.Add(std::move(m));
    }
    builder.BuildInto(this);
  }

  /// @brief Returns the number of sub-manifolds in this product.
  /// @return The total count of individual (uncompacted) sub-manifolds.
  [[nodiscard]] std::size_t NumSubManifolds() const noexcept { return size_; }

  /// @brief Returns the ambient size of the sub-manifold defined by tangent slices.
  ///
  /// Sums the ambient sizes of the sub-manifolds selected by each block slice,
  /// queried cheaply without constructing them.
  ///
  /// @copydetails Manifold_::AmbientSize(view)
  [[nodiscard]] std::size_t AmbientSize(beluga_estimation::view<const contiguous_slice> tangent_dims) const override {
    auto block_slices = MapBlockSlices(tangent_dims);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "invalid tangent_dims");
    std::size_t total = 0;
    for (const auto& block : *block_slices) {
      total += *block.ambient_slice.end - *block.ambient_slice.start;
    }
    return total;
  }

  /// @brief Returns the total ambient size of the sub-manifolds identified by index.
  ///
  /// This is a cheap query that avoids constructing the sub-manifolds, useful
  /// for sizing ambient-space buffers before an index-based @ref LiftAmbient.
  ///
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  /// @return The sum of the selected sub-manifolds' ambient sizes.
  /// @throws std::invalid_argument if @p indices is empty.
  /// @throws std::out_of_range if any index is not a valid sub-manifold index.
  [[nodiscard]] std::size_t AmbientSize(std_future::span<const std::size_t> indices) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    std::size_t total = 0;
    for (const auto& block : *block_slices) {
      total += *block.ambient_slice.end - *block.ambient_slice.start;
    }
    return total;
  }

  /// @brief Returns the total tangent size of the sub-manifolds identified by index.
  ///
  /// This is a cheap query that avoids constructing the sub-manifolds, useful
  /// for sizing tangent-space buffers (covariances, Jacobians) before an
  /// index-based @ref LiftTangent.
  ///
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  /// @return The sum of the selected sub-manifolds' tangent sizes.
  /// @throws std::invalid_argument if @p indices is empty.
  /// @throws std::out_of_range if any index is not a valid sub-manifold index.
  [[nodiscard]] std::size_t TangentSize(std_future::span<const std::size_t> indices) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    std::size_t total = 0;
    for (const auto& block : *block_slices) {
      total += *block.tangent_slice.end - *block.tangent_slice.start;
    }
    return total;
  }

  // Bring the base-class Lift/Embed overload sets into scope. The index-based
  // overrides below would otherwise hide the slice-based overloads declared in
  // Manifold_.
  using Manifold_<T>::AmbientSize;
  using Manifold_<T>::GetSubManifold;
  using Manifold_<T>::LiftTangent;
  using Manifold_<T>::EmbedTangent;
  using Manifold_<T>::LiftAmbient;
  using Manifold_<T>::EmbedAmbient;

  /// @brief Lifts a sub-manifold ambient vector into the full ambient space.
  ///
  /// Copies each block slice's ambient values into their parent ambient
  /// range. Non-selected blocks are left untouched, allowing callers to
  /// accumulate a full state vector across multiple lifts.
  ///
  /// @copydetails Manifold_::LiftAmbient
  void LiftAmbient(
      std_future::span<const T> sub_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> full_ambient) const override {
    auto block_slices = MapBlockSlices(tangent_dims);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "invalid tangent_dims");
    BELUGA_ESTIMATION_ASSERT(full_ambient.size() == ambient_size_);
    std::size_t sub_offset = 0;
    for (const auto& block : *block_slices) {
      const std::size_t ambient_size = *block.ambient_slice.end - *block.ambient_slice.start;
      auto dst = full_ambient.subspan(*block.ambient_slice.start, ambient_size);
      auto src = sub_ambient.subspan(sub_offset, ambient_size);
      std::copy(src.begin(), src.end(), dst.begin());
      sub_offset += ambient_size;
    }
    BELUGA_ESTIMATION_ASSERT(sub_offset == sub_ambient.size());
  }

  /// @brief Embeds (extracts) a sub-manifold ambient vector from the full ambient space.
  ///
  /// Copies each block slice's parent ambient range into the output,
  /// concatenated in selection order.
  ///
  /// @copydetails Manifold_::EmbedAmbient
  void EmbedAmbient(
      std_future::span<const T> full_ambient,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::span<T> sub_ambient) const override {
    auto block_slices = MapBlockSlices(tangent_dims);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "invalid tangent_dims");
    BELUGA_ESTIMATION_ASSERT(full_ambient.size() == ambient_size_);
    std::size_t sub_offset = 0;
    for (const auto& block : *block_slices) {
      const std::size_t ambient_size = *block.ambient_slice.end - *block.ambient_slice.start;
      auto dst = sub_ambient.subspan(sub_offset, ambient_size);
      auto src = full_ambient.subspan(*block.ambient_slice.start, ambient_size);
      std::copy(src.begin(), src.end(), dst.begin());
      sub_offset += ambient_size;
    }
    BELUGA_ESTIMATION_ASSERT(sub_offset == sub_ambient.size());
  }

  /// @brief Lifts a sub-manifold tangent vector identified by index into the full tangent space.
  ///
  /// @param index Zero-based position of the sub-manifold in the product
  ///   (0 is the first sub-manifold), not a tangent-space dimension.
  ///
  /// Routes to the multi-index overload with a single-element index span.
  void LiftTangent(std_future::span<const T> sub_tangent, std::size_t index, std_future::span<T> full_tangent) const {
    LiftTangent(sub_tangent, std_future::span<const std::size_t>(&index, 1), full_tangent);
  }

  /// @brief Embeds (extracts) a sub-manifold tangent vector identified by index from the full tangent space.
  ///
  /// @param index Zero-based position of the sub-manifold in the product
  ///   (0 is the first sub-manifold), not a tangent-space dimension.
  ///
  /// Routes to the multi-index overload with a single-element index span.
  void EmbedTangent(std_future::span<const T> full_tangent, std::size_t index, std_future::span<T> sub_tangent) const {
    EmbedTangent(full_tangent, std_future::span<const std::size_t>(&index, 1), sub_tangent);
  }

  /// @brief Lifts a sub-manifold tangent matrix identified by index into the full tangent space.
  ///
  /// @param index Zero-based position of the sub-manifold in the product
  ///   (0 is the first sub-manifold), not a tangent-space dimension.
  ///
  /// Routes to the multi-index overload with a single-element index span.
  void LiftTangent(std_future::mdspan<const T, 2> sub_matrix, std::size_t index, std_future::mdspan<T, 2> full_matrix)
      const {
    LiftTangent(sub_matrix, std_future::span<const std::size_t>(&index, 1), full_matrix);
  }

  /// @brief Embeds (extracts) a sub-manifold tangent matrix identified by index from the full tangent space.
  ///
  /// @param index Zero-based position of the sub-manifold in the product
  ///   (0 is the first sub-manifold), not a tangent-space dimension.
  ///
  /// Routes to the multi-index overload with a single-element index span.
  void EmbedTangent(std_future::mdspan<const T, 2> full_matrix, std::size_t index, std_future::mdspan<T, 2> sub_matrix)
      const {
    EmbedTangent(full_matrix, std_future::span<const std::size_t>(&index, 1), sub_matrix);
  }

  /// @brief Lifts a sub-manifold ambient vector identified by index into the full ambient space.
  ///
  /// @param index Zero-based position of the sub-manifold in the product
  ///   (0 is the first sub-manifold), not a tangent-space dimension.
  ///
  /// Routes to the multi-index overload with a single-element index span.
  void LiftAmbient(std_future::span<const T> sub_ambient, std::size_t index, std_future::span<T> full_ambient) const {
    LiftAmbient(sub_ambient, std_future::span<const std::size_t>(&index, 1), full_ambient);
  }

  /// @brief Embeds (extracts) a sub-manifold ambient vector identified by index from the full ambient space.
  ///
  /// @param index Zero-based position of the sub-manifold in the product
  ///   (0 is the first sub-manifold), not a tangent-space dimension.
  ///
  /// Routes to the multi-index overload with a single-element index span.
  void EmbedAmbient(std_future::span<const T> full_ambient, std::size_t index, std_future::span<T> sub_ambient) const {
    EmbedAmbient(full_ambient, std_future::span<const std::size_t>(&index, 1), sub_ambient);
  }

  /// @brief Lifts a set of sub-manifold tangent vectors into the full tangent space.
  ///
  /// The input holds the selected sub-manifolds' tangent vectors concatenated
  /// in the order they appear in @p indices; non-selected dimensions are left untouched.
  ///
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  ///
  /// @copydetails Manifold_::LiftTangent(view)
  void LiftTangent(
      std_future::span<const T> sub_tangent,
      std_future::span<const std::size_t> indices,
      std_future::span<T> full_tangent) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    std::size_t sub_offset = 0;
    for (const auto& block : *block_slices) {
      const std::size_t tangent_size = *block.tangent_slice.end - *block.tangent_slice.start;
      BELUGA_ESTIMATION_ASSERT(sub_tangent.size() >= sub_offset + tangent_size);
      auto dst = full_tangent.subspan(*block.tangent_slice.start, tangent_size);
      auto src = sub_tangent.subspan(sub_offset, tangent_size);
      std::copy(src.begin(), src.end(), dst.begin());
      sub_offset += tangent_size;
    }
    BELUGA_ESTIMATION_ASSERT(sub_offset == sub_tangent.size());
  }

  /// @brief Embeds (extracts) a set of sub-manifold tangent vectors from the full tangent space.
  ///
  /// The output holds the selected sub-manifolds' tangent vectors concatenated
  /// in the order they appear in @p indices.
  ///
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  ///
  /// @copydetails Manifold_::EmbedTangent(view)
  void EmbedTangent(
      std_future::span<const T> full_tangent,
      std_future::span<const std::size_t> indices,
      std_future::span<T> sub_tangent) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    std::size_t sub_offset = 0;
    for (const auto& block : *block_slices) {
      const std::size_t tangent_size = *block.tangent_slice.end - *block.tangent_slice.start;
      BELUGA_ESTIMATION_ASSERT(sub_tangent.size() >= sub_offset + tangent_size);
      auto dst = sub_tangent.subspan(sub_offset, tangent_size);
      auto src = full_tangent.subspan(*block.tangent_slice.start, tangent_size);
      std::copy(src.begin(), src.end(), dst.begin());
      sub_offset += tangent_size;
    }
    BELUGA_ESTIMATION_ASSERT(sub_offset == sub_tangent.size());
  }

  /// @brief Embeds (extracts) a set of sub-manifold tangent matrices from the full tangent space.
  ///
  /// The output holds the selected sub-manifolds' tangent matrices stacked by
  /// rows in the order they appear in @p indices.
  ///
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  ///
  /// @copydetails Manifold_::EmbedTangent(view)
  void EmbedTangent(
      std_future::mdspan<const T, 2> full_matrix,
      std_future::span<const std::size_t> indices,
      std_future::mdspan<T, 2> sub_matrix) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    // The same selection is applied to rows and columns, extracting square
    // sub-blocks from the diagonal.
    std::size_t sub_offset = 0;
    for (const auto& block : *block_slices) {
      const std::size_t tangent_offset = *block.tangent_slice.start;
      const std::size_t tangent_size = *block.tangent_slice.end - *block.tangent_slice.start;
      BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(0) >= sub_offset + tangent_size);
      BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(1) >= sub_offset + tangent_size);
      auto dst = std_future::submdspan(
          sub_matrix, std::pair{sub_offset, sub_offset + tangent_size},
          std::pair{sub_offset, sub_offset + tangent_size});
      auto src = std_future::submdspan(
          full_matrix, std::pair{tangent_offset, tangent_offset + tangent_size},
          std::pair{tangent_offset, tangent_offset + tangent_size});
      std_future::copy(dst, src);
      sub_offset += tangent_size;
    }
    BELUGA_ESTIMATION_ASSERT(sub_offset == sub_matrix.extent(0));
    BELUGA_ESTIMATION_ASSERT(sub_offset == sub_matrix.extent(1));
  }

  /// @brief Lifts a set of sub-manifold tangent matrices into the full tangent space.
  ///
  /// With no @p axis, the selection applies to both axes, placing square
  /// sub-blocks on the diagonal. With an axis (0 = rows, 1 = columns, see
  /// @ref kRowsAxis and @ref kColsAxis), the selection maps only that axis and
  /// the other axis passes through unchanged, so the input and output matrices
  /// share that axis's extent. This supports rectangular Jacobian blocks whose
  /// unselected axis spans dimensions unknown to the manifold (e.g.,
  /// measurement residuals).
  ///
  /// @param sub_matrix Input matrix with one row per selected row dimension and
  ///   one column per selected column dimension.
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  /// @param full_matrix Output matrix over the full tangent space.
  /// @param axis Which axis the selection maps; `std::nullopt` selects both.
  /// @throws std::invalid_argument if @p indices is empty.
  /// @throws std::out_of_range if any index is not a valid sub-manifold index.
  void LiftTangent(
      std_future::mdspan<const T, 2> sub_matrix,
      std_future::span<const std::size_t> indices,
      std_future::mdspan<T, 2> full_matrix,
      std::optional<std::size_t> axis = std::nullopt) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    LiftTangent(sub_matrix, *block_slices, full_matrix, axis);
  }

  /// @brief Lifts a sub-manifold tangent matrix into the full tangent space.
  ///
  /// The no-axis form is provided by the base-class three-argument overload;
  /// this overload requires an explicit @p axis (0 = rows, 1 = columns).
  ///
  /// @param sub_matrix Input matrix with one row per selected row dimension and
  ///   one column per selected column dimension.
  /// @param tangent_dims Sorted, non-overlapping slices describing the subset
  ///   of tangent dimensions to lift.
  /// @param full_matrix Output matrix over the full tangent space.
  /// @param axis Which axis the selection maps; `std::nullopt` selects both.
  /// @throws std::invalid_argument if @p tangent_dims is empty or invalid.
  void LiftTangent(
      std_future::mdspan<const T, 2> sub_matrix,
      beluga_estimation::view<const contiguous_slice> tangent_dims,
      std_future::mdspan<T, 2> full_matrix,
      std::optional<std::size_t> axis) const {
    auto block_slices = MapBlockSlices(tangent_dims);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "invalid tangent_dims");
    LiftTangent(sub_matrix, *block_slices, full_matrix, axis);
  }

  /// @brief Lifts a sub-manifold tangent matrix into the full tangent space.
  ///
  /// Maps the input rows through @p row_indices and its columns through
  /// @p col_indices, placing the tiles at the corresponding positions of the
  /// full matrix; non-selected entries are left untouched. This supports
  /// rectangular Jacobian blocks whose row and column subspaces select
  /// different sub-manifolds (e.g., process model Jacobians).
  ///
  /// @param sub_matrix Input matrix with one row per selected row sub-manifold
  ///   tangent dimension and one column per selected column sub-manifold
  ///   tangent dimension.
  /// @param row_indices Zero-based positions of the sub-manifolds to lift (rows),
  ///   in selection order. These are NOT tangent-space dimensions.
  /// @param col_indices Zero-based positions of the sub-manifolds to lift
  ///   (columns), in selection order. These are NOT tangent-space dimensions.
  /// @param full_matrix Output matrix over the full tangent space.
  /// @throws std::invalid_argument if @p row_indices or @p col_indices is empty.
  /// @throws std::out_of_range if any index is not a valid sub-manifold index.
  void LiftTangent(
      std_future::mdspan<const T, 2> sub_matrix,
      std_future::span<const std::size_t> row_indices,
      std_future::span<const std::size_t> col_indices,
      std_future::mdspan<T, 2> full_matrix) const {
    auto row_slices = MapBlockSlices(row_indices);
    auto col_slices = MapBlockSlices(col_indices);
    BELUGA_ESTIMATION_DEMAND(
        row_slices.has_value() && col_slices.has_value(), "sub-manifold indices must not be empty");
    std::size_t sub_row = 0;
    std::size_t sub_col = 0;
    for (const auto& row_slice : *row_slices) {
      const std::size_t row_start = *row_slice.tangent_slice.start;
      const std::size_t row_size = *row_slice.tangent_slice.end - *row_slice.tangent_slice.start;
      sub_col = 0;
      for (const auto& col_slice : *col_slices) {
        const std::size_t col_start = *col_slice.tangent_slice.start;
        const std::size_t col_size = *col_slice.tangent_slice.end - *col_slice.tangent_slice.start;
        BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(0) >= sub_row + row_size);
        BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(1) >= sub_col + col_size);
        auto dst = std_future::submdspan(
            full_matrix, std::pair{row_start, row_start + row_size}, std::pair{col_start, col_start + col_size});
        auto src = std_future::submdspan(
            sub_matrix, std::pair{sub_row, sub_row + row_size}, std::pair{sub_col, sub_col + col_size});
        std_future::copy(dst, src);
        sub_col += col_size;
      }
      sub_row += row_size;
    }
    BELUGA_ESTIMATION_ASSERT(sub_row == sub_matrix.extent(0));
    BELUGA_ESTIMATION_ASSERT(sub_col == sub_matrix.extent(1));
  }

  /// @brief Lifts a set of sub-manifold ambient vectors into the full ambient space.
  ///
  /// The input holds the selected sub-manifolds' ambient vectors concatenated
  /// in the order they appear in @p indices; non-selected blocks are left
  /// untouched.
  ///
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  ///
  /// @copydetails Manifold_::LiftAmbient(view)
  void LiftAmbient(
      std_future::span<const T> sub_ambient,
      std_future::span<const std::size_t> indices,
      std_future::span<T> full_ambient) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    std::size_t sub_offset = 0;
    for (const auto& block : *block_slices) {
      const std::size_t ambient_size = *block.ambient_slice.end - *block.ambient_slice.start;
      BELUGA_ESTIMATION_ASSERT(sub_ambient.size() >= sub_offset + ambient_size);
      auto dst = full_ambient.subspan(*block.ambient_slice.start, ambient_size);
      auto src = sub_ambient.subspan(sub_offset, ambient_size);
      std::copy(src.begin(), src.end(), dst.begin());
      sub_offset += ambient_size;
    }
    BELUGA_ESTIMATION_ASSERT(sub_offset == sub_ambient.size());
  }

  /// @brief Embeds (extracts) a set of sub-manifold ambient vectors from the full ambient space.
  ///
  /// The output holds the selected sub-manifolds' ambient vectors concatenated
  /// in the order they appear in @p indices.
  ///
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  ///
  /// @copydetails Manifold_::EmbedAmbient(view)
  void EmbedAmbient(
      std_future::span<const T> full_ambient,
      std_future::span<const std::size_t> indices,
      std_future::span<T> sub_ambient) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    std::size_t sub_offset = 0;
    for (const auto& block : *block_slices) {
      const std::size_t ambient_size = *block.ambient_slice.end - *block.ambient_slice.start;
      BELUGA_ESTIMATION_ASSERT(sub_ambient.size() >= sub_offset + ambient_size);
      auto dst = sub_ambient.subspan(sub_offset, ambient_size);
      auto src = full_ambient.subspan(*block.ambient_slice.start, ambient_size);
      std::copy(src.begin(), src.end(), dst.begin());
      sub_offset += ambient_size;
    }
    BELUGA_ESTIMATION_ASSERT(sub_offset == sub_ambient.size());
  }

  /// @brief Returns a view over the selected sub-manifolds' ambient vectors.
  ///
  /// Each element of the returned view is a span referencing the ambient
  /// values of one selected sub-manifold, in the order they appear in
  /// @p indices. The spans alias @p full_ambient and the view also references
  /// @p indices, so both must outlive the returned view; the view itself does
  /// not own storage, so it is safe to create and consume it within a single
  /// scope.
  ///
  /// @param full_ambient Ambient vector of the product manifold.
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  /// @return A view of spans, one per selected sub-manifold.
  /// @throws std::invalid_argument if @p indices is empty.
  /// @throws std::out_of_range if any index is not a valid sub-manifold index.
  [[nodiscard]] beluga_estimation::view<std_future::span<const T>> EmbedAmbient(
      std_future::span<const T> full_ambient,
      std_future::span<const std::size_t> indices) const {
    auto block_slices = MapBlockSlices(indices);
    BELUGA_ESTIMATION_DEMAND(block_slices.has_value(), "sub-manifold indices must not be empty");
    BELUGA_ESTIMATION_ASSERT(full_ambient.size() == ambient_size_);
    return beluga_estimation::view<std_future::span<const T>>{
        indices | ranges::views::transform([this, full_ambient](std::size_t index) {
          const auto it = std::partition_point(blocks_.begin(), blocks_.end(), [index](const Block& block) {
            return block.offset + block.size <= index;
          });
          BELUGA_ESTIMATION_ASSERT(it != blocks_.end(), "sub-manifold index out of range");
          const auto& block = *it;
          const std::size_t k = index - block.offset;
          const std::size_t ambient_size = block.manifold->AmbientSize();
          return full_ambient.subspan(block.ambient_offset + k * ambient_size, ambient_size);
        })};
  }

  /// @brief Returns the sub-manifold at a given index.
  /// @param index Zero-based index of the sub-manifold.
  /// @return Shared pointer to the sub-manifold.
  /// @throws std::out_of_range if @p index >= NumSubManifolds().
  [[nodiscard]] std::shared_ptr<const Manifold_<T>> GetSubManifold(std::size_t index) const {
    BELUGA_ESTIMATION_DEMAND(index < size_, "ProductManifold_::GetSubManifold(): index out of range");
    const auto it = std::partition_point(
        blocks_.begin(), blocks_.end(), [index](const Block& block) { return block.offset + block.size <= index; });
    BELUGA_ESTIMATION_ASSERT(it != blocks_.end(), "out of range");
    return it->manifold;
  }

  /// @brief Returns the total dimension of the ambient space.
  ///
  /// This is the sum of `AmbientSize()` across all sub-manifolds, multiplied
  /// by their respective counts.
  [[nodiscard]] std::size_t AmbientSize() const override { return ambient_size_; }

  /// @brief Returns the total dimension of the tangent space.
  ///
  /// This is the sum of `TangentSize()` across all sub-manifolds, multiplied
  /// by their respective counts.
  [[nodiscard]] std::size_t TangentSize() const override { return tangent_size_; }

  /// @brief Applies the Plus ($\oplus$) operation component-wise.
  ///
  /// For each sub-manifold block, delegates to its `Plus()` method with the
  /// corresponding ambient and tangent sub-views. The Jacobian (if provided)
  /// is filled as a block-diagonal matrix matching the block structure.
  ///
  /// @copydoc Manifold_::Plus
  void Plus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> /*hessian*/ = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(lhs.size() == ambient_size_);
    BELUGA_ESTIMATION_ASSERT(rhs.size() == tangent_size_);
    BELUGA_ESTIMATION_ASSERT(result.size() == ambient_size_);

    for (const auto& block : blocks_) {
      const std::size_t ambient_size = block.manifold->AmbientSize();
      const std::size_t tangent_size = block.manifold->TangentSize();
      for (std::size_t k = 0; k < block.size; ++k) {
        const std::size_t ambient_offset = block.ambient_offset + k * ambient_size;
        const std::size_t tangent_offset = block.tangent_offset + k * tangent_size;

        block.manifold->Plus(
            lhs.subspan(ambient_offset, ambient_size), rhs.subspan(tangent_offset, tangent_size),
            result.subspan(ambient_offset, ambient_size),
            GetJacobianBlock(jacobian, ambient_offset, tangent_offset, ambient_size, tangent_size), std::nullopt);
      }
    }
  }

  /// @brief Applies the Minus ($\ominus$) operation component-wise.
  ///
  /// For each sub-manifold block, delegates to its `Minus()` method with the
  /// corresponding ambient sub-views. The Jacobian (if provided) is filled
  /// as a block-diagonal matrix matching the block structure.
  ///
  /// @copydoc Manifold_::Minus
  void Minus(
      std_future::span<const T> lhs,
      std_future::span<const T> rhs,
      std_future::span<T> result,
      std::optional<std_future::mdspan<T, 2>> jacobian = std::nullopt,
      std::optional<std_future::mdspan<T, 2>> /*hessian*/ = std::nullopt) const override {
    BELUGA_ESTIMATION_ASSERT(lhs.size() == ambient_size_);
    BELUGA_ESTIMATION_ASSERT(rhs.size() == ambient_size_);
    BELUGA_ESTIMATION_ASSERT(result.size() == tangent_size_);

    for (const auto& block : blocks_) {
      const std::size_t ambient_size = block.manifold->AmbientSize();
      const std::size_t tangent_size = block.manifold->TangentSize();
      for (std::size_t k = 0; k < block.size; ++k) {
        const std::size_t ambient_off = block.ambient_offset + k * ambient_size;
        const std::size_t tangent_off = block.tangent_offset + k * tangent_size;

        block.manifold->Minus(
            lhs.subspan(ambient_off, ambient_size), rhs.subspan(ambient_off, ambient_size),
            result.subspan(tangent_off, tangent_size),
            GetJacobianBlock(jacobian, tangent_off, ambient_off, tangent_size, ambient_size), std::nullopt);
      }
    }
  }

  /// @brief Extracts a sub-manifold by selecting tangent dimensions.
  ///
  /// Uses @ref MapBlockSlices to resolve the requested tangent dimensions
  /// into block-aligned slices. When a slice fully covers a block's tangent
  /// space, the original block manifold is reused; otherwise the sub-manifold
  /// is extracted via the block's own `GetSubManifold()`.
  ///
  /// @copydoc Manifold_::GetSubManifold(beluga_estimation::view)
  [[nodiscard]] std::shared_ptr<const Manifold_<T>> GetSubManifold(
      beluga_estimation::view<const contiguous_slice> tangent_dims) const override {
    auto blocks = MapBlockSlices(tangent_dims);
    if (!blocks) {
      return nullptr;
    }
    ProductManifoldBuilder_<T> builder;
    for (const auto& block : *blocks) {
      const std::size_t slice_size = *block.tangent_slice.end - *block.tangent_slice.start;
      if (slice_size != block.manifold->TangentSize()) {
        auto submanifold = block.manifold->GetSubManifold(ToLocalTangentSlice(block));
        if (!submanifold) {
          return nullptr;
        }
        builder.Add(std::move(submanifold));
      } else {
        builder.Add(block.manifold->shared_from_this());
      }
    }
    return builder.Build();
  }

 private:
  friend class ProductManifoldBuilder_<T>;

  ProductManifold_() = default;

  /// @brief Returns a block view of the Jacobian.
  /// @param jacobian The full Jacobian matrix (row-major layout).
  /// @param row_offset The starting row index of the block view.
  /// @param col_offset The starting column index of the block view.
  /// @param row_extent The number of rows in the block view.
  /// @param col_extent The number of columns in the block view.
  /// @return Block view of the Jacobian.
  static std::optional<std_future::mdspan<T, 2>> GetJacobianBlock(
      const std::optional<std_future::mdspan<T, 2>>& jacobian,
      std::size_t row_offset,
      std::size_t col_offset,
      std::size_t row_extent,
      std::size_t col_extent) {
    if (!jacobian.has_value()) {
      return std::nullopt;
    }
    return std_future::submdspan(
        *jacobian, std::pair{row_offset, row_offset + row_extent}, std::pair{col_offset, col_offset + col_extent});
  }

  /// @brief A run-length compressed block of identical sub-manifolds.
  struct Block {
    /// The sub-manifold repeated in this block.
    std::shared_ptr<const Manifold_<T>> manifold;
    /// Number of consecutive identical sub-manifolds in this block.
    std::size_t size = 0;
    /// Index of the first sub-manifold in this block (sub-manifold coordinate).
    std::size_t offset = 0;
    /// Total ambient dimension contributed by this block.
    std::size_t ambient_size = 0;
    /// Starting offset in the global ambient vector.
    std::size_t ambient_offset = 0;
    /// Total tangent dimension contributed by this block.
    std::size_t tangent_size = 0;
    /// Starting offset in the global tangent vector.
    std::size_t tangent_offset = 0;
  };

  /// @brief A slice of a block produced by @ref MapBlockSlices.
  struct BlockSlice {
    /// The sub-manifold that owns this slice (non-owning pointer).
    ///
    /// Use @ref Manifold_::shared_from_this to obtain a shared pointer.
    const Manifold_<T>* manifold;
    /// Tangent-space slice in the parent tangent vector.
    contiguous_slice tangent_slice;
    /// Ambient-space slice in the parent ambient vector.
    contiguous_slice ambient_slice;
  };

  /// @brief Maps tangent-space dimension slices to block-aligned slices.
  ///
  /// Validates that @p tangent_dims are within bounds and non-overlapping,
  /// then resolves each slice into one or more @ref BlockSlice entries.
  /// A single slice may span multiple blocks or partial blocks — each
  /// contiguous portion is returned as a separate slice. The slices carry
  /// their tangent and ambient ranges in the parent vectors' coordinates.
  ///
  /// @param tangent_dims Non-overlapping slices describing the subset of
  ///   tangent dimensions to map.
  /// @return A vector of block slices, or `std::nullopt` if any slice is
  ///   out of bounds or overlapping, or selects a partial sub-manifold of a
  ///   manifold without proper sub-manifolds (e.g., SE(2)).
  [[nodiscard]] std::optional<std::vector<BlockSlice>> MapBlockSlices(
      beluga_estimation::view<const contiguous_slice> tangent_dims) const {
    // Validate non-empty, bounds and non-overlap.
    if (tangent_dims.size() == 0) {
      return std::nullopt;
    }
    if (!beluga_estimation::disjoint(tangent_dims)) {
      return std::nullopt;
    }
    for (const auto& dims : tangent_dims) {
      const std::size_t start_dim = dims.start.value_or(0);
      const std::size_t end_dim = dims.end.value_or(tangent_size_);
      if (start_dim >= end_dim || end_dim > tangent_size_) {
        return std::nullopt;
      }
    }
    std::vector<BlockSlice> slices;
    for (const auto& dims : tangent_dims) {
      std::size_t start_dim = dims.start.value_or(0);
      std::size_t end_dim = dims.end.value_or(tangent_size_);

      while (start_dim < end_dim) {
        // Blocks are sorted by offset, so look for the first block
        // that is not strictly below the current slice, which happens
        // to be the block that contains the start of the slice.
        const auto it = std::partition_point(blocks_.begin(), blocks_.end(), [&](const Block& block) {
          return block.tangent_offset + block.tangent_size <= start_dim;
        });
        BELUGA_ESTIMATION_ASSERT(it != blocks_.end(), "slice start is out of bounds");
        const auto& block = *it;
        const std::size_t slice_start = (start_dim - block.tangent_offset) % block.manifold->TangentSize();
        const std::size_t slice_size = std::min(end_dim - start_dim, block.manifold->TangentSize() - slice_start);
        // Reject partial selections of manifolds without proper sub-manifolds
        // (i.e., non-Euclidean manifolds, detected by AmbientSize() !=
        // TangentSize()), for which the ambient range is ill-defined. Full
        // selections remain valid. This also covers nested product blocks that
        // contain non-Euclidean components, which are not supported for
        // partial selections.
        if (slice_size != block.manifold->TangentSize() &&
            block.manifold->AmbientSize() != block.manifold->TangentSize()) {
          return std::nullopt;
        }
        const std::size_t repetition = (start_dim - block.tangent_offset) / block.manifold->TangentSize();
        // The ambient range in parent coordinates: full blocks span the whole
        // repetition; partial (Euclidean) blocks span the selected
        // coordinates. This assumes that, for manifolds supporting proper
        // sub-manifolds, the sub-manifold ambient is the selected coordinates
        // (i.e., ambient and tangent indices coincide), which holds for the
        // Euclidean manifolds in this codebase.
        const std::size_t ambient_within_start = (slice_size == block.manifold->TangentSize()) ? 0 : slice_start;
        const std::size_t ambient_size =
            (slice_size == block.manifold->TangentSize()) ? block.manifold->AmbientSize() : slice_size;
        const std::size_t ambient_offset =
            block.ambient_offset + repetition * block.manifold->AmbientSize() + ambient_within_start;
        slices.push_back({
            block.manifold.get(),
            contiguous_slice::Span(start_dim, slice_size),
            contiguous_slice::Span(ambient_offset, ambient_size),
        });
        start_dim += slice_size;
      }
    }
    return slices;
  }

  /// @brief Maps sub-manifold indices to their block slices.
  ///
  /// Each index resolves to the full tangent and ambient ranges of its
  /// sub-manifold, in the parent vectors' coordinates.
  ///
  /// @param indices Zero-based positions of the sub-manifolds in the product
  ///   (0 is the first sub-manifold), in selection order. These are NOT
  ///   tangent-space dimensions.
  /// @return A vector of block slices, or `std::nullopt` if @p indices is empty.
  /// @throws std::out_of_range if any index is not a valid sub-manifold index.
  [[nodiscard]] std::optional<std::vector<BlockSlice>> MapBlockSlices(
      std_future::span<const std::size_t> indices) const {
    if (indices.size() == 0) {
      return std::nullopt;
    }
    std::vector<BlockSlice> slices;
    slices.reserve(indices.size());
    for (const std::size_t index : indices) {
      const auto it = std::partition_point(
          blocks_.begin(), blocks_.end(), [index](const Block& block) { return block.offset + block.size <= index; });
      BELUGA_ESTIMATION_DEMAND(
          it != blocks_.end(), "ProductManifold_::MapBlockSlices: sub-manifold index out of range");
      const auto& block = *it;
      const std::size_t k = index - block.offset;
      slices.push_back({
          block.manifold.get(),
          contiguous_slice::Span(
              block.tangent_offset + k * block.manifold->TangentSize(), block.manifold->TangentSize()),
          contiguous_slice::Span(
              block.ambient_offset + k * block.manifold->AmbientSize(), block.manifold->AmbientSize()),
      });
    }
    return slices;
  }

  /// @brief Lifts a sub-manifold tangent matrix into the full tangent space
  /// from pre-resolved block slices.
  ///
  /// Private overload sharing the implementation of the public axis-based
  /// @ref LiftTangent overloads; @p block_slices must come from
  /// @ref MapBlockSlices. With no @p axis, the same selection is applied to
  /// rows and columns, placing square sub-blocks on the diagonal. With an axis
  /// (0 = rows, 1 = columns), the selection maps only that axis and the other
  /// axis passes through unchanged: for rows the input and output share their
  /// column extent, and for columns they share their row extent.
  void LiftTangent(
      std_future::mdspan<const T, 2> sub_matrix,
      const std::vector<BlockSlice>& block_slices,
      std_future::mdspan<T, 2> full_matrix,
      std::optional<std::size_t> axis) const {
    if (!axis.has_value()) {
      std::size_t sub_offset = 0;
      for (const auto& block : block_slices) {
        const std::size_t tangent_offset = *block.tangent_slice.start;
        const std::size_t tangent_size = *block.tangent_slice.end - *block.tangent_slice.start;
        BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(0) >= sub_offset + tangent_size);
        BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(1) >= sub_offset + tangent_size);
        auto dst = std_future::submdspan(
            full_matrix, std::pair{tangent_offset, tangent_offset + tangent_size},
            std::pair{tangent_offset, tangent_offset + tangent_size});
        auto src = std_future::submdspan(
            sub_matrix, std::pair{sub_offset, sub_offset + tangent_size},
            std::pair{sub_offset, sub_offset + tangent_size});
        std_future::copy(dst, src);
        sub_offset += tangent_size;
      }
      BELUGA_ESTIMATION_ASSERT(sub_offset == sub_matrix.extent(0));
      BELUGA_ESTIMATION_ASSERT(sub_offset == sub_matrix.extent(1));
      return;
    }
    if (*axis == kRowsAxis) {
      BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(1) == full_matrix.extent(1));
      std::size_t sub_row = 0;
      for (const auto& block : block_slices) {
        const std::size_t row_start = *block.tangent_slice.start;
        const std::size_t row_size = *block.tangent_slice.end - *block.tangent_slice.start;
        BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(0) >= sub_row + row_size);
        auto dst = std_future::submdspan(
            full_matrix, std::pair{row_start, row_start + row_size}, std::pair{0, full_matrix.extent(1)});
        auto src = std_future::submdspan(
            sub_matrix, std::pair{sub_row, sub_row + row_size}, std::pair{0, sub_matrix.extent(1)});
        std_future::copy(dst, src);
        sub_row += row_size;
      }
      BELUGA_ESTIMATION_ASSERT(sub_row == sub_matrix.extent(0));
    } else {
      BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(0) == full_matrix.extent(0));
      std::size_t sub_col = 0;
      for (const auto& block : block_slices) {
        const std::size_t col_start = *block.tangent_slice.start;
        const std::size_t col_size = *block.tangent_slice.end - *block.tangent_slice.start;
        BELUGA_ESTIMATION_ASSERT(sub_matrix.extent(1) >= sub_col + col_size);
        auto dst = std_future::submdspan(
            full_matrix, std::pair{0, full_matrix.extent(0)}, std::pair{col_start, col_start + col_size});
        auto src = std_future::submdspan(
            sub_matrix, std::pair{0, sub_matrix.extent(0)}, std::pair{sub_col, sub_col + col_size});
        std_future::copy(dst, src);
        sub_col += col_size;
      }
      BELUGA_ESTIMATION_ASSERT(sub_col == sub_matrix.extent(1));
    }
  }

  /// @brief Converts the parent tangent range of a block slice to the owning
  /// block manifold's tangent range.
  [[nodiscard]] contiguous_slice ToLocalTangentSlice(const BlockSlice& slice) const {
    const auto it = std::partition_point(blocks_.begin(), blocks_.end(), [&](const Block& block) {
      return block.tangent_offset + block.tangent_size <= *slice.tangent_slice.start;
    });
    BELUGA_ESTIMATION_ASSERT(it != blocks_.end(), "slice start is out of bounds");
    const std::size_t within_start = (*slice.tangent_slice.start - it->tangent_offset) % it->manifold->TangentSize();
    return contiguous_slice::Span(within_start, *slice.tangent_slice.end - *slice.tangent_slice.start);
  }

  /// The compacted blocks forming this product manifold.
  std::vector<Block> blocks_;
  /// Total ambient dimension (sum of all block ambient sizes).
  std::size_t ambient_size_ = 0;
  /// Total tangent dimension (sum of all block tangent sizes).
  std::size_t tangent_size_ = 0;
  /// Total number of individual (uncompacted) sub-manifolds.
  std::size_t size_ = 0;
};

/// @brief Builder for @ref ProductManifold_ with run-length compaction.
///
/// Accumulates sub-manifolds and compacts consecutive identical ones
/// into single blocks. Call @ref Add() one or more times, then @ref Build()
/// to produce the final @ref ProductManifold_.
///
/// @tparam T The numeric scalar type.
template <typename T>
class ProductManifoldBuilder_ {
 public:
  /// @brief Adds a sub-manifold (or repeated copies) to the product.
  ///
  /// If the last added manifold is the same object as @p manifold, the
  /// block's count is incremented instead of creating a new block (run-length
  /// compaction).
  ///
  /// @param manifold The sub-manifold to add.
  /// @param count Number of consecutive copies of @p manifold. Must be >= 1.
  /// @throws std::invalid_argument if @p count < 1.
  void Add(std::shared_ptr<const Manifold_<T>> manifold, std::size_t count = 1) {
    BELUGA_ESTIMATION_DEMAND(count >= 1, "count must be >= 1");
    if (!blocks_.empty()) {
      auto& last_block = blocks_.back();
      if (last_block.manifold == manifold) {
        last_block.size += count;
        return;
      }
    }
    blocks_.push_back({std::move(manifold), count});
  }

  /// @brief Builds the @ref ProductManifold_ and returns it.
  ///
  /// Finalizes layout offsets and transfers ownership of the compacted
  /// blocks to a new @ref ProductManifold_ instance.
  ///
  /// @return A shared pointer to the constructed product manifold.
  /// @throws std::runtime_error if no manifolds have been added.
  std::shared_ptr<ProductManifold_<T>> Build() {
    BELUGA_ESTIMATION_DEMAND(!blocks_.empty(), "called on empty builder");
    auto result = std::shared_ptr<ProductManifold_<T>>(new ProductManifold_<T>());
    BuildInto(result.get());
    return result;
  }

 private:
  /// @brief Finalizes block layout and transfers data into a product manifold.
  ///
  /// Computes ambient/tangent offsets and sub-manifold indices for each
  /// block, then moves the block vector into the product manifold instance.
  ///
  /// @param manifold Pointer to the product manifold to initialize.
  void BuildInto(ProductManifold_<T>* manifold) {
    BELUGA_ESTIMATION_ASSERT(manifold != nullptr);
    std::size_t ambient_size = 0;
    std::size_t tangent_size = 0;
    std::size_t size = 0;
    for (auto& block : blocks_) {
      block.ambient_size = block.manifold->AmbientSize() * block.size;
      block.tangent_size = block.manifold->TangentSize() * block.size;
      block.ambient_offset = ambient_size;
      block.tangent_offset = tangent_size;
      block.offset = size;
      ambient_size += block.ambient_size;
      tangent_size += block.tangent_size;
      size += block.size;
    }
    manifold->blocks_ = std::move(blocks_);
    manifold->ambient_size_ = ambient_size;
    manifold->tangent_size_ = tangent_size;
    manifold->size_ = size;
  }

  using Block = typename ProductManifold_<T>::Block;
  /// The accumulated (but not yet finalized) blocks.
  std::vector<Block> blocks_;
};

using ProductManifold = ProductManifold_<DefaultScalar>;

}  // namespace beluga_estimation
