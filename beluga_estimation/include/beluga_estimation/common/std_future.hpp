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
 * @brief Includes utilities that were defined in the STL after C++17.
 */

#include <atomic>
#include <cassert>
#include <cstring>
#include <memory>
#include <type_traits>

#include <mdspan/mdspan.hpp>
#include <range/v3/view/span.hpp>

#include <beluga_estimation/common/assert.hpp>

namespace beluga_estimation::std_future {

/**
 * @brief Converts an enumeration to its underlying type.
 * @note Since C++23
 */
template <class Enum>
constexpr auto to_underlying(Enum v) noexcept {
  return static_cast<std::underlying_type_t<Enum>>(v);
}

/**
 * @brief Casts away constness from a pointer.
 */
template <class T>
constexpr T* as_mutable(const T* t) noexcept {
  return const_cast<T*>(t);
}

/**
 * @brief Casts away constness from a reference.
 */
template <class T>
constexpr T& as_mutable(const T& t) noexcept {
  return const_cast<T&>(t);
}

/**
 * @brief 1-dimensional span.
 * @note Since C++20
 */
template <typename T, auto N = ranges::dynamic_extent>
using span = ranges::span<T, N>;

/**
 * @brief N-dimensional span backed by layout_stride.
 *
 * Wrapper over Kokkos::mdspan with layout_stride that provides:
 * - Row-major-backed constructors for contiguous memory (backward compatible).
 * - An explicit-stride constructor for sub-block views.
 *
 * The row-major 2D constructor `mdspan(data, rows, cols)` creates a view
 * where element (r, c) is at `data[r * cols + c]`.
 */
template <typename T, size_t N>
class mdspan : public Kokkos::mdspan<T, Kokkos::dextents<size_t, N>, Kokkos::layout_stride> {
  using base_type = Kokkos::mdspan<T, Kokkos::dextents<size_t, N>, Kokkos::layout_stride>;

 public:
  using extents_type = typename base_type::extents_type;
  using mapping_type = typename base_type::mapping_type;
  /// @brief Default constructor.
  mdspan() = default;

  /// @brief Contiguous row-major 1D constructor.
  mdspan(T* data, std::size_t ext0)
      : base_type(data, mapping_type(extents_type(ext0), std::array<std::size_t, 1>{1})) {}

  /// @brief Contiguous row-major 2D constructor.
  mdspan(T* data, std::size_t ext0, std::size_t ext1)
      : base_type(data, mapping_type(extents_type(ext0, ext1), std::array<std::size_t, 2>{ext1, 1})) {}

  /// @brief Construct with an explicit mapping (strided sub-blocks).
  mdspan(T* data, const mapping_type& mapping) : base_type(data, mapping) {}

  /// @brief Implicit conversion from the underlying Kokkos mdspan.
  mdspan(const base_type& other)  // NOLINT(google-explicit-constructor)
      : base_type(other) {}

  /// @brief Implicit conversion from an mdspan with a convertible element type.
  ///
  /// Enables, e.g., passing a mutable `mdspan<double, 2>` where a
  /// `mdspan<const double, 2>` is expected, mirroring the behavior of spans.
  template <typename U, std::enable_if_t<std::is_convertible_v<U (*)[], T (*)[]>, int> = 0>
  mdspan(const mdspan<U, N>& other)  // NOLINT(google-explicit-constructor)
      : base_type(other) {}
};

/**
 * @brief Creates a sub-mdspan view.
 *
 * Forwards to Kokkos::submdspan and wraps the result in mdspan.
 * Supports the full range of slice specifiers (pairs, integers, slices, full_extent).
 */
template <class ElementType, class Extents, class LayoutPolicy, class AccessorPolicy, class... SliceSpecifiers>
constexpr auto submdspan(
    const Kokkos::mdspan<ElementType, Extents, LayoutPolicy, AccessorPolicy>& src,
    SliceSpecifiers... slices) {
  return mdspan<ElementType, Extents::rank()>(Kokkos::submdspan(src, std::move(slices)...));
}

/**
 * @brief Copies the contents of an mdspan into another.
 *
 * Uses `memmove` when the element type is trivially copyable and the layout
 * affords vectorization i.e. contiguous rows or contiguous columns. Falls
 * back to an element-wise copy otherwise.
 *
 * @warning Overlapping mdspans are not an error. However, only fully
 * contiguous layouts are guaranteed to produce the expected result when they
 * overlap. For slice or element-wise copies, overlaps may result in overwrites
 * to the source mdspan during copy. Callers must ensure such a copy is semantically
 * correct for their use case.
 *
 * @param dst The destination matrix.
 * @param src The source matrix.
 */
template <typename T>
void copy(std_future::mdspan<T, 2> dst, std_future::mdspan<const T, 2> src) {
  BELUGA_ESTIMATION_ASSERT(dst.extent(0) == src.extent(0));
  BELUGA_ESTIMATION_ASSERT(dst.extent(1) == src.extent(1));
  const std::size_t num_rows = dst.extent(0);
  const std::size_t num_columns = dst.extent(1);
  if (num_rows == 0 || num_columns == 0) {
    return;
  }
  if constexpr (std::is_trivially_copyable_v<T>) {
    if (src.mapping().stride(1) == 1 && dst.mapping().stride(1) == 1) {
      // Row major optimized copy: mdspans may have contiguous rows.
      if (src.mapping().stride(0) == num_columns && dst.mapping().stride(0) == num_columns) {
        // Rows are contiguous: one memmove for the whole matrix.
        std::memmove(dst.data_handle(), src.data_handle(), num_rows * num_columns * sizeof(T));
      } else {
        // Rows may be strided and/or overlapping: one memmove per row, copy may be destructive.
        for (std::size_t i = 0; i < num_rows; ++i) {
          std::memmove(
              dst.data_handle() + i * dst.mapping().stride(0), src.data_handle() + i * src.mapping().stride(0),
              num_columns * sizeof(T));
        }
      }
      return;
    }
    if (src.mapping().stride(0) == 1 && dst.mapping().stride(0) == 1) {
      // Column major optimized copy: mdspans may have contiguous columns.
      if (src.mapping().stride(1) == num_rows && dst.mapping().stride(1) == num_rows) {
        // Columns are contiguous: one memmove for the whole matrix.
        std::memmove(dst.data_handle(), src.data_handle(), num_rows * num_columns * sizeof(T));
      } else {
        // Columns may be strided and/or overlapping: one memmove per column, copy may be destructive.
        for (std::size_t j = 0; j < num_columns; ++j) {
          std::memmove(
              dst.data_handle() + j * dst.mapping().stride(1), src.data_handle() + j * src.mapping().stride(1),
              num_rows * sizeof(T));
        }
      }
      return;
    }
  }
  for (std::size_t i = 0; i < num_rows; ++i) {
    for (std::size_t j = 0; j < num_columns; ++j) {
      dst(i, j) = src(i, j);
    }
  }
}

/**
 * @brief A standard C++ trick to make static_assert wait for template instantiation.
 */
template <typename...>
constexpr bool always_false_v = false;

namespace detail {

/// @brief Shared stop state for stop_source and stop_token.
struct stop_state {
  std::atomic<bool> stopped{false};
};

}  // namespace detail

class stop_source;

/**
 * @brief A token that can be checked to see if a stop has been requested.
 * @note Since C++20
 */
class stop_token {
 public:
  /// @brief Constructs a stop token that does not refer to any stop state.
  stop_token() noexcept = default;

  /// @brief Returns true if the associated stop_source has had stop requested.
  [[nodiscard]] bool stop_requested() const noexcept {
    return state_ && state_->stopped.load(std::memory_order_acquire);
  }

  /// @brief Returns true if this token is associated with a stop state.
  [[nodiscard]] bool stop_possible() const noexcept { return state_ != nullptr; }

  friend bool operator==(const stop_token& lhs, const stop_token& rhs) noexcept { return lhs.state_ == rhs.state_; }

  friend bool operator!=(const stop_token& lhs, const stop_token& rhs) noexcept { return !(lhs == rhs); }

 private:
  friend class stop_source;
  explicit stop_token(std::shared_ptr<detail::stop_state> state) noexcept : state_(std::move(state)) {}
  std::shared_ptr<detail::stop_state> state_;
};

/**
 * @brief A source that can issue stop requests to associated stop_tokens.
 * @note Since C++20
 */
class stop_source {
 public:
  /// @brief Tag type for constructing a stop_source with no associated state.
  struct nostopstate_t {
    explicit nostopstate_t() = default;
  };

  /// @brief Tag value for constructing a stop_source with no associated state.
  static constexpr nostopstate_t nostopstate{};

  /// @brief Constructs a stop_source with a new associated stop state.
  stop_source() : state_(std::make_shared<detail::stop_state>()) {}

  /// @brief Constructs a stop_source with no associated stop state.
  explicit stop_source(nostopstate_t) noexcept : state_(nullptr) {}

  /// @brief Returns a stop_token associated with this stop_source's stop state.
  [[nodiscard]] stop_token get_token() const noexcept { return stop_token{state_}; }

  /// @brief Issues a stop request.
  /// @return true if this call made the stop request, false if stop was already requested or not possible.
  bool request_stop() noexcept {
    if (!stop_possible()) {
      return false;
    }
    bool expected = false;
    return state_->stopped.compare_exchange_strong(expected, true, std::memory_order_acq_rel);
  }

  /// @brief Returns true if stop has been requested.
  [[nodiscard]] bool stop_requested() const noexcept {
    return state_ && state_->stopped.load(std::memory_order_acquire);
  }

  /// @brief Returns true if this stop_source has an associated stop state.
  [[nodiscard]] bool stop_possible() const noexcept { return state_ != nullptr; }

  friend bool operator==(const stop_source& lhs, const stop_source& rhs) noexcept { return lhs.state_ == rhs.state_; }

  friend bool operator!=(const stop_source& lhs, const stop_source& rhs) noexcept { return !(lhs == rhs); }

 private:
  std::shared_ptr<detail::stop_state> state_;
};

}  // namespace beluga_estimation::std_future
