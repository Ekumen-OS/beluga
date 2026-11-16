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
 * @brief Range utilities, including transpose_view and materialized_transpose_view.
 */

#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

#include <range/v3/range/access.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/traits.hpp>
#include <range/v3/view/interface.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/view.hpp>

namespace beluga_estimation {

/**
 * @brief A view that transposes a range of ranges.
 *
 * Given a range of ranges (the "columns"), yields a range where each element
 * is a lazy view over one element from each column at the current position.
 * Iteration stops when any column is exhausted (truncation to shortest).
 *
 * The column views are materialized into owned storage at construction.
 * Range-v3's type-erased view iterators (e.g., those of `any_view`) alias the
 * view's storage rather than owning it, so the column views must outlive the
 * iterators the transpose captures; owning them guarantees that regardless of
 * what the caller passes. The per-column sequences themselves are consumed
 * lazily: each column's begin iterator (and end sentinel) is moved into
 * internal storage on first iteration, and every subsequent operation advances
 * and dereferences those stored iterators. The underlying sequences are
 * therefore consumed exactly once, in order, which is compatible with
 * single-pass (input) columns. The transpose itself is single-pass when its
 * columns are.
 *
 * @warning Row references are valid only until the next increment of the
 *          returned iterator. For input ranges (e.g. `single_pass_view`),
 *          elements are consumed on advance; materialize the row explicitly
 *          if it needs to outlive the current iteration step.
 *
 * @tparam Rng A range of ranges; its elements (the column views) are moved
 *         into owned storage on construction.
 *
 * Usage:
 * @code
 *   std::vector<some_view> columns = {view1, view2, view3};
 *   for (auto row : transpose_view(std::move(columns))) {
 *     // row is a view over columns at their current iteration step:
 *     // - row[0] is the current item in column 0's iteration
 *     // - row[1] is the current item in column 1's iteration
 *     // - etc.
 *     // Row references are valid only during each iteration
 *     // e.g. when columns are input (i.e. consumable) ranges.
 *   }
 * @endcode
 */
template <typename Rng>
class transpose_view : public ranges::view_interface<transpose_view<Rng>> {
  using InnerRng = ranges::range_value_t<Rng>;

  // Owning, stable storage for the column views; see the class documentation.
  std::vector<InnerRng> columns_;
  bool initialized_ = false;
  std::vector<ranges::iterator_t<InnerRng>> its_;
  std::vector<ranges::sentinel_t<InnerRng>> ends_;

  /// @brief Captures, once, the begin and end of each column.
  ///
  /// Iterators are moved out of `begin()`, so move-only input iterators are
  /// supported; ends are stored as sentinels so exhaustion checks never
  /// re-iterate the underlying sequences.
  void EnsureInitialized() {
    if (!initialized_) {
      for (auto& col : columns_) {
        its_.push_back(ranges::begin(col));
        ends_.push_back(ranges::end(col));
      }
      initialized_ = true;
    }
  }

  /// @brief A row is exhausted when any column's stored iterator reached its end.
  bool AnyExhausted() {
    if (its_.empty()) {
      return true;  // no columns, no rows
    }
    for (std::size_t i = 0; i < its_.size(); ++i) {
      if (its_[i] == ends_[i]) {
        return true;
      }
    }
    return false;
  }

 public:
  /// @brief Construct from a range of ranges (columns).
  ///
  /// Materializes the columns into owned storage, so the captured iterators
  /// never outlive the column views regardless of what the caller passes.
  explicit transpose_view(Rng columns) {
    for (auto&& col : columns) {
      columns_.push_back(std::move(col));
    }
  }

  /**
   * @brief Input iterator over transposed rows.
   *
   * Dereferencing returns a lazy view over the current element of each
   * column.  The view references are valid only until this iterator is
   * incremented.
   *
   * Models `ranges::input_iterator`: it is equality-comparable with itself
   * (a default-constructed iterator is treated as the end sentinel) and
   * usable with range-v3 algorithms that consume single-pass ranges.
   */
  struct Iterator {
    /// @brief Default-constructs an iterator equivalent to the end sentinel.
    Iterator() : parent_(nullptr) {}

    /// @brief Constructs an iterator into the given view.
    explicit Iterator(transpose_view* parent) : parent_(parent) {}

    /// @brief Lazy view over the current row (one element per column).
    auto operator*() const {
      return parent_->its_ |
             ranges::views::transform([](auto& it) -> ranges::range_reference_t<InnerRng> { return *it; });
    }

    /// @brief Advance all columns by one position.
    Iterator& operator++() {
      for (auto& it : parent_->its_) {
        ++it;
      }
      return *this;
    }

    /// @brief Post-increment.  Required to model `ranges::weakly_incrementable`.
    Iterator operator++(int) {
      auto copy = *this;
      ++(*this);
      return copy;
    }

    /// @brief End-of-range comparison.  Returns true when any column is exhausted.
    bool operator==(const Iterator& other) const {
      if (parent_ == nullptr || other.parent_ == nullptr) {
        // Either iterator is the sentinel, check if the other
        // is exhausted or if both are sentinels.
        auto* live = parent_ != nullptr ? parent_ : other.parent_;
        return live == nullptr || live->AnyExhausted();
      }
      // View is consumable, iterators just consume it.
      // Two iterators that refer to the same view are
      // therefore equal.
      return parent_ == other.parent_;
    }
    bool operator!=(const Iterator& other) const { return !(*this == other); }

   private:
    transpose_view* parent_;

   public:
    // Iterator trait typedefs, declared after the members they reference so
    // that `value_type` (the lazily-produced row view) can be spelled with
    // `decltype`.  Needed for `ranges::input_iterator` conformance.
    using difference_type = std::ptrdiff_t;
    using value_type = decltype(std::declval<const Iterator&>().operator*());
    using reference = value_type;
    using pointer = void;
    using iterator_category = std::input_iterator_tag;
  };

  /// @brief Return an iterator to the first transposed row.
  Iterator begin() {
    EnsureInitialized();
    return Iterator{this};
  }

  /// @brief Return a sentinel iterator.
  Iterator end() { return Iterator{nullptr}; }
};

// ---------------------------------------------------------------------------
// materialized_transpose_view
// ---------------------------------------------------------------------------

/**
 * @brief A view that transposes a range of ranges, materializing each row.
 *
 * Behaves like @ref transpose_view, but each yielded row is an owned
 * `std::vector` copy of the columns' current elements instead of a lazy view
 * over them. Rows are materialized on demand: the first dereference of an
 * iterator position copies the current element of every column into an
 * internal buffer, and subsequent dereferences of the same position return
 * the cached buffer.
 *
 * Because each row is an owned copy, it does not alias the columns'
 * iterators: it can be iterated multiple times, passed to algorithms, or
 * stored in containers. The row reference is valid until the iterator is
 * incremented (the standard input-iterator contract), after which the buffer
 * is reused for the next row.
 *
 * The transpose itself remains single-pass when its columns are, matching
 * @ref transpose_view; the improvement is that each row is a stable,
 * fully-functional (forward) range.
 *
 * @warning Row references are valid only until the next increment of the
 *          returned iterator; the shared buffer is reused for the next row.
 *          Copy the row explicitly if it needs to outlive the current
 *          iteration step.
 *
 * @note Like @ref transpose_view, this view is not safe to copy after
 *       iteration has begun: the captured column iterators alias the
 *       original's storage.
 *
 * @tparam Rng A range of ranges; its elements (the column views) are moved
 *         into owned storage on construction.
 *
 * Usage:
 * @code
 *   std::vector<some_view> columns = {view1, view2, view3};
 *   for (const auto& row : materialized_transpose_view(std::move(columns))) {
 *     // row is a std::vector over the current elements (one per column)
 *     // row[0] is column 0's current element
 *     // row[1] is column 1's current element
 *     // etc.
 *     // row is an owned copy: valid until the next ++ of the outer loop
 *   }
 * @endcode
 */
template <typename Rng>
class materialized_transpose_view : public ranges::view_interface<materialized_transpose_view<Rng>> {
  using InnerRng = ranges::range_value_t<Rng>;
  using Row = std::vector<ranges::range_value_t<InnerRng>>;

  // The lazy transpose provides the column storage and iteration state.
  transpose_view<Rng> transpose_;
  // Shared row cache. The transpose is single-pass (all iterators share one
  // logical position), so a single buffer suffices; it is materialized on
  // first dereference and reused until the next increment.
  Row row_;
  bool row_valid_ = false;

 public:
  /// @brief Construct from a range of ranges (columns).
  ///
  /// Materializes the columns into owned storage, so the captured iterators
  /// never outlive the column views regardless of what the caller passes.
  explicit materialized_transpose_view(Rng columns) : transpose_(std::move(columns)) {}

  /**
   * @brief Input iterator over materialized transposed rows.
   *
   * Dereferencing returns a reference to an owned `std::vector` holding a
   * copy of the current element of each column. The reference is valid
   * until this iterator is incremented.
   *
   * Models `ranges::input_iterator`: it is equality-comparable with itself
   * (a default-constructed iterator is treated as the end sentinel) and
   * usable with range-v3 algorithms that consume single-pass ranges.
   */
  struct Iterator {
    /// @brief Default-constructs an iterator equivalent to the end sentinel.
    Iterator() : parent_(nullptr) {}

    /// @brief Constructs an iterator into the given view.
    explicit Iterator(materialized_transpose_view* parent)
        : parent_(parent), transpose_it_(parent->transpose_.begin()) {}

    /// @brief Reference to the materialized current row (one element per column).
    const Row& operator*() const {
      if (!parent_->row_valid_) {
        parent_->row_.clear();
        for (auto&& element : *transpose_it_) {
          parent_->row_.push_back(std::forward<decltype(element)>(element));
        }
        parent_->row_valid_ = true;
      }
      return parent_->row_;
    }

    /// @brief Advance all columns by one position.
    Iterator& operator++() {
      ++transpose_it_;
      parent_->row_valid_ = false;
      return *this;
    }

    /// @brief Post-increment.  Required to model `ranges::weakly_incrementable`.
    Iterator operator++(int) {
      auto copy = *this;
      ++(*this);
      return copy;
    }

    /// @brief End-of-range comparison.  Returns true when any column is exhausted.
    bool operator==(const Iterator& other) const { return transpose_it_ == other.transpose_it_; }
    bool operator!=(const Iterator& other) const { return !(*this == other); }

   private:
    materialized_transpose_view* parent_;
    typename transpose_view<Rng>::Iterator transpose_it_;

   public:
    // Iterator trait typedefs, declared after the members they reference so
    // that `value_type` (the materialized row) can be spelled with the
    // enclosing class's `Row` alias.  Needed for `ranges::input_iterator`
    // conformance.
    using difference_type = std::ptrdiff_t;
    using value_type = Row;
    using reference = const Row&;
    using pointer = void;
    using iterator_category = std::input_iterator_tag;
  };

  /// @brief Return an iterator to the first materialized row.
  Iterator begin() { return Iterator{this}; }

  /// @brief Return a sentinel iterator.
  Iterator end() { return Iterator{}; }
};

// ---------------------------------------------------------------------------
// Deduction guide
// ---------------------------------------------------------------------------
template <typename Rng>
explicit transpose_view(Rng&&) -> transpose_view<std::decay_t<Rng>>;

template <typename Rng>
explicit materialized_transpose_view(Rng&&) -> materialized_transpose_view<std::decay_t<Rng>>;

// ---------------------------------------------------------------------------
// Pipeable adaptor object
// ---------------------------------------------------------------------------
namespace detail {

/// @brief Function object that constructs a transpose_view from a range of ranges.
struct transpose_fn {
  template <typename Rng>
  auto operator()(Rng&& rng) const {
    return transpose_view<std::decay_t<Rng>>{std::forward<Rng>(rng)};
  }
};

/// @brief Function object that constructs a materialized_transpose_view from a range of ranges.
struct materialized_transpose_fn {
  template <typename Rng>
  auto operator()(Rng&& rng) const {
    return materialized_transpose_view<std::decay_t<Rng>>{std::forward<Rng>(rng)};
  }
};

}  // namespace detail

/// @brief Pipeable adaptor: `columns | beluga_estimation::views::transpose`.
namespace views {
inline constexpr ranges::views::view_closure<detail::transpose_fn> transpose{};

/// @brief Pipeable adaptor: `columns | beluga_estimation::views::materialized_transpose`.
inline constexpr ranges::views::view_closure<detail::materialized_transpose_fn> materialized_transpose{};
}  // namespace views

}  // namespace beluga_estimation
