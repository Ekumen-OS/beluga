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

#ifndef BELUGA_UTILITY_INDEXING_ITERATOR_HPP
#define BELUGA_UTILITY_INDEXING_ITERATOR_HPP

#include <cstdint>
#include <iterator>
#include <type_traits>

/**
 * \file
 * \brief Implementation of a random access iterator for indexable containers.
 */

namespace beluga {

/// A random access iterator for any indexable container.
/**
 * It can provide a pair of iterators for any container type that supports subscripting with an integral-like index.
 *
 * \tparam Indexable Container type, potentially const. It must support subscripting.
 * \tparam Index Container index type. It must be default constructible and integral-like.
 * Defaults to the indexable container size type, if defined.
 */
template <class Indexable, class Index = typename Indexable::size_type>
class IndexingIterator {
 public:
  /// Value type of the iterator.
  using value_type =
      std::remove_cv_t<std::remove_reference_t<decltype(std::declval<Indexable>()[std::declval<Index>()])>>;
  /// Value reference type of the iterator.
  using reference = decltype(std::declval<Indexable>()[std::declval<Index>()]);
  /// Value pointer type of the iterator.
  using pointer = decltype(std::addressof(std::declval<Indexable>()[std::declval<Index>()]));
  /// Signed difference type of the iterator.
  using difference_type = std::make_signed_t<decltype(std::declval<Index>() - std::declval<Index>())>;
  /// Category of the iterator.
  using iterator_category = std::random_access_iterator_tag;

  /// Default constructor. Iterator will point nowhere.
  explicit IndexingIterator() = default;

  /// Constructs iterator given a pointer to an `indexable` container and a `cursor` index on it.
  explicit IndexingIterator(Indexable* indexable, Index cursor = Index{})
      : indexable_{indexable}, cursor_{std::move(cursor)} {}

  /// Constructs iterator given a reference to an `indexable` container and a `cursor` index on it.
  explicit IndexingIterator(Indexable& indexable, Index cursor = Index{})
      : IndexingIterator(std::addressof(indexable), std::move(cursor)) {}

  /// Post-increments iterator position in the target container.
  IndexingIterator operator++(int) noexcept {
    IndexingIterator other = *this;
    ++(*this);
    return other;
  }

  /// Pre-increments iterator position in the target container.
  IndexingIterator& operator++() noexcept {
    ++cursor_;
    return *this;
  }

  /// Post-decrements iterator position in the target container.
  IndexingIterator operator--(int) noexcept {
    IndexingIterator other = *this;
    --(*this);
    return other;
  }

  /// Pre-decrements iterator position in the target container.
  IndexingIterator& operator--() noexcept {
    --cursor_;
    return *this;
  }

  /// Forwards iterator position a given `offset`, in-place.
  IndexingIterator& operator+=(difference_type offset) noexcept {
    using raw_difference_type = decltype(cursor_ - cursor_);
    if (offset < 0) {
      cursor_ -= static_cast<raw_difference_type>(-offset);
    } else {
      cursor_ += static_cast<raw_difference_type>(offset);
    }
    return *this;
  }

  /// Forwards iterator position a given `offset`, yielding a modified copy.
  [[nodiscard]] IndexingIterator operator+(difference_type offset) const noexcept {
    IndexingIterator other = *this;
    other += offset;
    return other;
  }

  /// Forwards `iterator` a given `offset`, yielding a modified copy.
  [[nodiscard]] friend IndexingIterator operator+(difference_type offset, const IndexingIterator& iterator) {
    return iterator + offset;
  }

  /// Rewinds iterator position a given `offset`, in-place.
  IndexingIterator& operator-=(difference_type offset) noexcept {
    using raw_difference_type = decltype(cursor_ - cursor_);
    if (offset < 0) {
      cursor_ += static_cast<raw_difference_type>(-offset);
    } else {
      cursor_ -= static_cast<raw_difference_type>(offset);
    }
    return *this;
  }

  /// Rewinds iterator position a given `offset`, yielding a modified copy.
  [[nodiscard]] IndexingIterator operator-(difference_type offset) const noexcept {
    IndexingIterator other = *this;
    other -= offset;
    return other;
  }

  /// Computes the difference (i.e. the distance) between the positions of this iterator and another.
  [[nodiscard]] difference_type operator-(const IndexingIterator& other) const noexcept {
    if (cursor_ < other.cursor_) {
      return -static_cast<difference_type>(other.cursor_ - cursor_);
    }
    return static_cast<difference_type>(cursor_ - other.cursor_);
  }

  /// Dereferences iterator at a given `offset` from its current position.
  /**
   * Behavior is undefined for offsets that take the iterator past the limits of the target container.
   */
  [[nodiscard]] reference operator[](difference_type offset) const noexcept { return (*indexable_)[cursor_ + offset]; }

  /// Dereferences iterator at its current position.
  /**
   * Behavior is undefined if the iterator is past the limits of the target container.
   */
  [[nodiscard]] reference operator*() const noexcept { return (*indexable_)[cursor_]; }

  /// Dereferences iterator at its current position and yields a pointer to it.
  /**
   * Behavior is undefined if the iterator is past the limits of the target container.
   */
  [[nodiscard]] pointer operator->() const noexcept { return std::addressof((*indexable_)[cursor_]); }

  /// Checks if iterator position is strictly before that of another.
  /**
   * Can only be true for iterators that have the same target container.
   */
  bool operator<(const IndexingIterator& other) const noexcept {
    return indexable_ == other.indexable_ && cursor_ < other.cursor_;
  }

  /// Checks if iterator position is before or equal to that of another.
  /**
   * Can only be true for iterators that have the same target container.
   */
  bool operator<=(const IndexingIterator& other) const noexcept {
    return indexable_ == other.indexable_ && cursor_ <= other.cursor_;
  }

  /// Checks if iterator position is strictly after that of another.
  /**
   * Can only be true for iterators that have the same target container.
   */
  bool operator>(const IndexingIterator& other) const noexcept { return !((*this) <= other); }

  /// Checks if iterator position is after or equal to that of another.
  /**
   * Can only be true for iterators that have the same target container.
   */
  bool operator>=(const IndexingIterator& other) const noexcept { return !((*this) < other); }

  /// Checks if iterator position is equal to that of another.
  /**
   * Can only be true for iterators that have the same target container.
   */
  bool operator==(const IndexingIterator& other) const noexcept {
    return indexable_ == other.indexable_ && cursor_ == other.cursor_;
  }

  /// Checks if iterator position is not equal to that of another.
  bool operator!=(const IndexingIterator& other) const noexcept { return !(*this == other); }

 private:
  Indexable* indexable_{nullptr};
  Index cursor_{};
};

}  // namespace beluga

#endif
