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

#ifndef BELUGA_CONTAINERS_CIRCULAR_ARRAY_HPP
#define BELUGA_CONTAINERS_CIRCULAR_ARRAY_HPP

#include <array>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <beluga/utility/indexing_iterator.hpp>

/**
 * \file
 * \brief Implementation of a generic circular array container.
 */

namespace beluga {

/// Feature flags for circular arrays.
enum class CircularArrayFeatureFlags : int {
  kNone = 0x00,
  kRolloverOnWrite = 0x01,    ///<! If enabled, older values in the array are overwritten by
                              /// newer values if the array has reached its maximum size already.
  kExtrapolateOnRead = 0x02,  ///<! If enabled, the back value is extrapolated for constant
                              /// accesses up to the array maximum size.
  kLayoutReversal = 0x04      ///<! If enabled, the circular array memory layout is reversed so that
                              /// values can be pushed to the front rather than the back of the array.
};

/// Bitwise OR operator overload to combine two feature flags in a single mask-like flag.
inline constexpr CircularArrayFeatureFlags operator|(CircularArrayFeatureFlags lflag, CircularArrayFeatureFlags rflag) {
  return static_cast<CircularArrayFeatureFlags>(
      static_cast<typename std::underlying_type<CircularArrayFeatureFlags>::type>(lflag) |
      static_cast<typename std::underlying_type<CircularArrayFeatureFlags>::type>(rflag));
}

/// Bitwise AND operator overload to check of the presence of a feature `flag` in a feature `mask`.
inline constexpr bool operator&(CircularArrayFeatureFlags mask, CircularArrayFeatureFlags flag) {
  return static_cast<bool>(
      static_cast<typename std::underlying_type<CircularArrayFeatureFlags>::type>(mask) &
      static_cast<typename std::underlying_type<CircularArrayFeatureFlags>::type>(flag));
}

/// An implementation of generic, non-threadsafe circular array.
/**
 * Modelled after `std::array`. Its maximum size is fixed at compile-time. Most operations are O(1)
 * but for a few that exhibit worse case O(N) (not considering the complexity of move-construction
 * and move-assignment of array elements of type T). Additionally, it features optional indexing
 * reversal (making it a LIFO rather than a FIFO data structure), optional back value extrapolation
 * for constant read accesses, and optional rollover on write accesses (i.e. overwriting the oldest
 * value when pushing to an array instance that is full).
 *
 * \tparam T Element type.
 *   It must be [default constructible](https://en.cppreference.com/w/cpp/named_req/DefaultConstructible),
 *   [move constructible](https://en.cppreference.com/w/cpp/named_req/MoveConstructible), and
 *   [move assignable](https://en.cppreference.com/w/cpp/named_req/MoveAssignable).
 * \tparam N Maximum size. It must be a positive integer.
 * \tparam F Array feature flags, to enable optional functionality. It defaults to none.
 */
template <typename T, std::size_t N, CircularArrayFeatureFlags F = CircularArrayFeatureFlags::kNone>
class CircularArray {
 public:
  /// Value type of the array.
  using value_type = T;
  /// Size type of the array.
  using size_type = std::size_t;
  /// Size difference type of the array.
  using difference_type = std::ptrdiff_t;
  /// Value reference type of the array.
  using reference = value_type&;
  /// Constant value reference type of the array.
  using const_reference = const value_type&;
  /// Value pointer type of the array.
  using pointer = value_type*;
  /// Constant value pointer type of the arra.y
  using const_pointer = const value_type*;

  /// Allocator type of the array (required in range-v3 10.0).
  using allocator_type = void;

  /// Iterator type of the array.
  using iterator = IndexingIterator<CircularArray<T, N, F>>;
  /// Constant iterator type of the array.
  using const_iterator = IndexingIterator<const CircularArray<T, N, F>>;
  /// Reverse iterator type of the array.
  using reverse_iterator = std::reverse_iterator<iterator>;
  /// Constant reverse iterator type of the array.
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  /// Default constructor.
  /**
   * Array will be functionally empty but storage will be default initialized.
   */
  CircularArray() = default;

  /// Constructs array from a pair of iterators.
  /**
   * Functionally equivalent to repeated push_back() operations
   * or push_front() operations in reverse if the layout reversal
   * feature is enabled. That is, both range and array layouts match.
   *
   * \param first Iterator to the beginning of a range.
   * \param last Sentinel for the end of a range.
   * \throws std::length_error If the range does not fit in the array
   * (and the rollover on write feature is not enabled).
   */
  template <
      typename Iterator,
      typename Sentinel,
      typename = std::enable_if_t<std::is_same_v<T, typename std::iterator_traits<Iterator>::value_type>>>
  CircularArray(Iterator first, Sentinel last) {
    if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
      tail_index_ = N - 1;
      for (size_ = 0; size_ < N && first != last; ++size_, ++first) {
        data_[tail_index_ - size_] = *first;
      }
      if constexpr (!(F & CircularArrayFeatureFlags::kRolloverOnWrite)) {
        if (first != last) {
          throw std::length_error{"Range does not fit in circular array"};
        }
      }
    } else {
      const auto n = static_cast<size_type>(std::distance(first, last));
      if (n > N) {
        if constexpr (!(F & CircularArrayFeatureFlags::kRolloverOnWrite)) {
          throw std::length_error{"Range does not fit in circular array"};
        } else {
          std::advance(first, n - N);
        }
      }
      for (size_ = 0; size_ < N && first != last; ++size_, ++first) {
        data_[size_] = *first;
      }
      tail_index_ = size_ - 1;
    }
  }

  /// Constructs array from an aggregate.
  /**
   * Functionally equivalent to repeated push_back() operations or push_front()
   * operations in reverse if the layout reversal feature is enabled. That is,
   * both aggregate and array layouts match.
   *
   * \param data Aggregate data to initialize the array with. Its size must be
   * in the [1, N] interval.
   */
  template <std::size_t M, typename = std::enable_if_t<M >= 1 && M <= N>>
  explicit CircularArray(T (&&data)[M]) : tail_index_(M - 1), size_(M) {  // NOLINT(modernize-avoid-c-arrays)
    if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
      for (std::size_t i = 0; i < M; ++i)
        data_[i] = data[M - i - 1];
    } else {
      for (std::size_t i = 0; i < M; ++i)
        data_[i] = data[i];
    }
  }

  /// Returns an iterator pointing to the front of the array.
  [[nodiscard]] constexpr iterator begin() noexcept { return iterator(this); }
  /// Returns a constant iterator pointing to the front of the array.
  [[nodiscard]] constexpr const_iterator begin() const noexcept { return const_iterator(this); }
  /// Returns a constant iterator pointing to the front of the array.
  [[nodiscard]] constexpr const_iterator cbegin() const noexcept { return const_iterator(this); }

  /// Returns an iterator pointing past the back of the array.
  [[nodiscard]] constexpr iterator end() noexcept { return iterator(this, size()); }
  /// Returns a constant iterator pointing past the back of the array.
  [[nodiscard]] constexpr const_iterator end() const noexcept { return const_iterator(this, effective_size()); }
  /// Returns a constant iterator pointing past the back of the array.
  [[nodiscard]] constexpr const_iterator cend() const noexcept { return const_iterator(this, effective_size()); }

  /// Returns a reverse iterator pointing to the back of the array.
  [[nodiscard]] constexpr reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }
  /// Returns a constant reverse iterator pointing to the back of the array.
  [[nodiscard]] constexpr const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }
  /// Returns a constant reverse iterator pointing to the back of the array.
  [[nodiscard]] constexpr const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(cend()); }

  /// Returns a reverse iterator pointing past the front of the array.
  [[nodiscard]] constexpr reverse_iterator rend() noexcept { return reverse_iterator(begin()); }
  /// Returns a constant reverse iterator pointing past the front of the array.
  [[nodiscard]] constexpr const_reverse_iterator rend() const noexcept { return const_reverse_iterator(begin()); }
  /// Returns a constant reverse iterator pointing past the front of the array.
  [[nodiscard]] constexpr const_reverse_iterator crend() const noexcept { return const_reverse_iterator(cbegin()); }

  /// Pushes a `value` to the back of the array.
  /**
   * Only available when the layout reversal feature is not enabled.
   *
   * \throws std::length_error If the array is full and
   * the rollover on write feature is not enabled.
   */
  template <bool Enabled = !(F & CircularArrayFeatureFlags::kLayoutReversal)>
  std::enable_if_t<Enabled> push_back(value_type value) {
    static_assert(
        !(F & CircularArrayFeatureFlags::kLayoutReversal),
        "Cannot push_back() when the layout reversal feature is enabled.");
    if constexpr (!(F & CircularArrayFeatureFlags::kRolloverOnWrite)) {
      if (size_ == N) {
        throw std::length_error{"Circular array reached its maximum size"};
      }
    }
    if (++tail_index_ == N)
      tail_index_ = 0;
    data_[tail_index_] = std::move(value);
    size_ = std::min(size_ + 1, N);
  }

  /// Pushes a `value` at the front of the array.
  /**
   * Only available when the layout reversal feature is enabled.
   *
   * \throws std::length_error If the array is full and
   * the rollover on write feature is not enabled.
   */
  template <bool Enabled = F& CircularArrayFeatureFlags::kLayoutReversal>
  std::enable_if_t<Enabled> push_front(value_type value) {
    static_assert(
        F & CircularArrayFeatureFlags::kLayoutReversal,
        "Cannot push_front() when the layout reversal feature is not enabled.");
    if constexpr (!(F & CircularArrayFeatureFlags::kRolloverOnWrite)) {
      if (size_ == N) {
        throw std::length_error{"Circular array reached its maximum size"};
      }
    }
    if (++tail_index_ == N)
      tail_index_ = 0;
    data_[tail_index_] = std::move(value);
    size_ = std::min(size_ + 1, N);
  }

  /// Pops a `value` from the back of the array.
  /**
   * Only available when the layout reversal feature is enabled.
   * Behavior is undefined when popping from the back of an empty array.
   * No destructors are invoked on the value popped.
   */
  template <bool Enabled = F& CircularArrayFeatureFlags::kLayoutReversal>
  std::enable_if_t<Enabled> pop_back() noexcept {
    --size_;
  }

  /// Pops a `value` from the front of the array.
  /**
   * Only available when the layout reversal feature is not enabled.
   * Behavior is undefined when popping from the front of an empty array.
   * No destructors are invoked on the value popped.
   */
  template <bool Enabled = !(F & CircularArrayFeatureFlags::kLayoutReversal)>
  std::enable_if_t<Enabled> pop_front() noexcept {
    --size_;
  }

  /// Returns a reference to the value at the back of the array.
  /**
   * Behavior is undefined when accessing the back of an empty array.
   */
  [[nodiscard]] constexpr reference back() noexcept {
    if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
      return data_[head_index()];
    } else {
      return data_[tail_index_];
    }
  }
  /// Returns a constant reference to the value at the back of the array.
  /**
   * Behavior is undefined when accessing the back of an empty array.
   */
  [[nodiscard]] constexpr const_reference back() const noexcept {
    if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
      return data_[head_index()];
    } else {
      return data_[tail_index_];
    }
  }

  /// Returns a reference to the value at the front of the array.
  /**
   * Behavior is undefined when accessing the front of an empty array.
   */
  [[nodiscard]] constexpr reference front() noexcept {
    if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
      return data_[tail_index_];
    } else {
      return data_[head_index()];
    }
  }

  /// Returns a constant reference to the value at the front of the array.
  /**
   * Behavior is undefined when accessing the front of an empty array.
   */
  [[nodiscard]] constexpr const_reference front() const noexcept {
    if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
      return data_[tail_index_];
    } else {
      return data_[head_index()];
    }
  }

  /// Returns a reference to the array value at the given `index`.
  /**
   * \throws std::out_of_range If `index` is past the array's effective size.
   */
  [[nodiscard]] constexpr reference at(size_type index) {
    if (index >= effective_size()) {
      throw std::out_of_range{"Index out of circular array range"};
    }
    return (*this)[index];
  }

  /// Returns a constant reference to the array value at the given `index`.
  /**
   * \throws std::out_of_range If `index` is past the array effective size.
   */
  [[nodiscard]] constexpr const_reference at(size_type index) const {
    if (index >= effective_size()) {
      throw std::out_of_range{"Index out of circular array range"};
    }
    return (*this)[index];
  }

  /// Returns a reference to the array value at the given `index`.
  /**
   * Behavior is undefined when `index` is greater than or equal to the array size.
   */
  [[nodiscard]] constexpr reference operator[](size_type index) noexcept {
    if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
      return data_[static_cast<size_type>(index > tail_index_) * N + tail_index_ - index];
    } else {
      return data_[static_cast<size_type>(size_ - index > tail_index_ + 1) * N + tail_index_ - (size_ - index - 1)];
    }
  }

  /// Returns a constant reference to the array value at the given `index`.
  /**
   * Behavior is undefined when `index` is greater than or equal to the array effective size.
   */
  [[nodiscard]] constexpr const_reference operator[](size_type index) const noexcept {
    if constexpr (F & CircularArrayFeatureFlags::kExtrapolateOnRead) {
      index = std::min(index, size_ - 1);
    }
    if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
      return data_[static_cast<size_type>(index > tail_index_) * N + tail_index_ - index];
    } else {
      return data_[static_cast<size_type>(size_ - index > tail_index_ + 1) * N + tail_index_ - (size_ - index - 1)];
    }
  }

  /// Fills array to its maximum size with a given `value`.
  /**
   * Functionally equivalent to repeated push_back() operations
   * or push_front() operations if the layout reversal feature is
   * enabled until the array reaches its maximum size. Existing values
   * are kept. Filling an array that is full is a no-op.
   */
  void fill(const T& value) {
    while (size_ < N) {
      tail_index_ = (tail_index_ + 1) % N;
      data_[tail_index_] = value;
      ++size_;
    }
  }

  /// Clears array.
  /**
   * No value destructors are invoked.
   */
  void clear() noexcept { size_ = 0; }

  /// Swaps array with another.
  /**
   * Arrays being swapped may not necessarily have the same features enabled.
   * This is most relevant when arrays differ in layout direction, as swapping
   * two such arrays effectively reverses their elements.
   */
  template <CircularArrayFeatureFlags G>
  void swap(CircularArray<T, N, G>& other) noexcept(std::is_nothrow_swappable_v<T>) {
    using std::swap;
    swap(data_, other.data_);
    swap(size_, other.size_);
    swap(tail_index_, other.tail_index_);
  }

  template <typename U, std::size_t M, CircularArrayFeatureFlags G>
  friend class CircularArray;

  /// Returns a pointer to the underlying array data.
  /**
   * Note that a circular array does not feature a contiguous layout in memory.
   * Indices and distances between them for the array do not map to its data.
   */
  [[nodiscard]] constexpr T* data() noexcept { return data_.data(); }

  /// Returns a constant pointer to the underlying array data.
  /**
   * Note that a circular array does not feature a contiguous layout in memory.
   * Indices and distances between them for the array do not map to its data.
   */
  [[nodiscard]] constexpr const T* data() const noexcept { return data_.data(); }

  /// Returns true if the array is full, false otherwise.
  [[nodiscard]] constexpr bool full() const noexcept { return size_ == N; }

  /// Returns true if the array is empty, false otherwise.
  [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0U; }

  /// Returns the current array size.
  [[nodiscard]] constexpr size_type size() const noexcept { return size_; }

  /// Returns the maximum array size.
  [[nodiscard]] constexpr size_type max_size() const noexcept { return N; }

  /// Returns the effective array size.
  /**
   * Nominally, the effective array size is equal to the array size. If the last
   * value extrapolation feature is enabled, however, the effective array size
   * can only be 0 or N, when the array is empty and when the array is non-empty
   * respectively.
   */
  [[nodiscard]] constexpr size_type effective_size() const noexcept {
    if constexpr (F & CircularArrayFeatureFlags::kExtrapolateOnRead) {
      return static_cast<size_type>(size_ > 0) * N;
    } else {
      return size_;
    }
  }

 private:
  // Computes the head index of the array.
  [[nodiscard]] constexpr size_type head_index() const noexcept {
    return static_cast<size_type>(size_ > tail_index_ + 1) * N + tail_index_ - size_ + 1;
  }

  std::array<T, N> data_;
  size_type tail_index_{0U};
  size_type size_{0U};
};

/// Convenient type alias for a circular array that behaves like a rolling window.
/**
 * A rolling window automatically overwrites older values, it extrapolates its last
 * value to always seem full, and it is accessed by the front.
 */
template <typename T, std::size_t N>
using RollingWindow = CircularArray<
    T,
    N,
    CircularArrayFeatureFlags::kRolloverOnWrite | CircularArrayFeatureFlags::kExtrapolateOnRead |
        CircularArrayFeatureFlags::kLayoutReversal>;

/// Convenient stream operator overload to push a `value` to a circular `array`.
/**
 * Functionally equivalent to a push_back() (or push_front() if the layout
 * reversal feature is enabled).
 */
template <typename T, std::size_t N, CircularArrayFeatureFlags F>
CircularArray<T, N, F>& operator<<(CircularArray<T, N, F>& array, T value) {
  if constexpr (F & CircularArrayFeatureFlags::kLayoutReversal) {
    array.push_front(std::move(value));
  } else {
    array.push_back(std::move(value));
  }
  return array;
}

/// Gets an lvalue reference to the ith value in a given `array`.
/**
 * This is an `std::get` overload that relies on argument-dependent lookup (ADL).
 */
template <std::size_t I, class T, std::size_t N, CircularArrayFeatureFlags F>
constexpr T& get(CircularArray<T, N, F>& array) noexcept {
  return array[I];
}

/// Gets an rvalue reference to the ith value in a given `array`.
/**
 * This is an `std::get` overload that relies on argument-dependent lookup (ADL).
 */
template <std::size_t I, class T, std::size_t N, CircularArrayFeatureFlags F>
constexpr T&& get(CircularArray<T, N, F>&& array) noexcept {
  return array[I];
}

/// Gets a constant lvalue reference to the ith value in a given `array`.
/**
 * This is an `std::get` overload that relies on argument-dependent lookup (ADL).
 */
template <std::size_t I, class T, std::size_t N, CircularArrayFeatureFlags F>
constexpr const T& get(const CircularArray<T, N, F>& array) noexcept {
  return array[I];
}

/// Gets a constant rvalue reference to the ith value in a given `array`.
/**
 * This is an `std::get` overload that relies on argument-dependent lookup (ADL).
 */
template <std::size_t I, class T, std::size_t N, CircularArrayFeatureFlags F>
constexpr const T&& get(const CircularArray<T, N, F>&& array) noexcept {
  return array[I];
}

/// Swaps arrays `a` and `b`.
/**
 * This is an `std::swap` overload that relies on argument-dependent lookup (ADL).
 */
template <typename T, std::size_t N, CircularArrayFeatureFlags F, CircularArrayFeatureFlags G>
constexpr void swap(CircularArray<T, N, F>& a, CircularArray<T, N, G>& b) {
  a.swap(b);
}

}  // namespace beluga

namespace std {

/// `std::tuple_size` specialization for circular arrays.
template <typename T, std::size_t N, beluga::CircularArrayFeatureFlags F>
struct tuple_size<beluga::CircularArray<T, N, F>> : std::integral_constant<std::size_t, N> {};

/// `std::tuple_element` specialization for circular arrays.
template <std::size_t I, typename T, std::size_t N, beluga::CircularArrayFeatureFlags F>
struct tuple_element<I, beluga::CircularArray<T, N, F>> {
  using type = T;  ///<! Always T since circular arrays are homogeneous.
};

}  // namespace std

#endif
