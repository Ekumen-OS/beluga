// Copyright 2022-2024 Ekumen, Inc.
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

#ifndef BELUGA_TUPLE_VECTOR_HPP
#define BELUGA_TUPLE_VECTOR_HPP

#include <tuple>
#include <vector>

#include <beluga/type_traits.hpp>
#include <beluga/views/zip.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/iterator/insert_iterators.hpp>
#include <range/v3/view/const.hpp>
#include <range/v3/view/take.hpp>

/**
 * \file
 * \brief Implementation of a tuple of containers.
 */

namespace beluga {

/// Primary template for a tuple of containers.
template <template <class> class InternalContainer, class T>
class TupleContainer;

/// An implementation of a tuple of containers, with an interface that looks like a container of tuples.
/**
 * i.e. though this is implemented internally as an `std::tuple<InternalContainer<Types>...>`, the interface looks like
 * an `InternalContainer<std::tuple<Types>...>`.
 * It provides the convenience of the second, but when iterating over only one of the elements of the tuple in the
 * container it has better performance (because of cache locality).
 *
 * \tparam InternalContainer Container type, e.g. std::vector.
 * \tparam ...Types Elements types of the tuple.
 */
template <template <class> class InternalContainer, class... Types>
class TupleContainer<InternalContainer, std::tuple<Types...>> {
 public:
  /// Value type of the container.
  using value_type = std::tuple<Types...>;

  /// Reference type of the container.
  using reference_type = ranges::common_tuple<Types&...>;

  /// Size type of the container.
  using size_type = std::size_t;

  /// Difference type of the container.
  using difference_type = std::ptrdiff_t;

  /// Allocator type.
  /**
   * This alias needs to exist to satisfy `ranges::to` container concept checks.
   * It is not actually used, and there is no meaninful type we can specify here.
   * See https://github.com/ericniebler/range-v3/blob/0.10.0/include/range/v3/range/conversion.hpp#L262
   */
  using allocator_type = void;

  /// Default constructor, will default initialize all containers in the tuple.
  constexpr TupleContainer() = default;

  /// Constructs a container of size count, all values are default initialized.
  /**
   * \param count Size of the container.
   */
  explicit constexpr TupleContainer(size_type count) : sequences_{((void)sizeof(Types), count)...} {}

  /// Constructs a container from iterators.
  template <typename I, typename S>
  constexpr TupleContainer(I first, S last) {
    assign(first, last);
  }

  /// Constructs a container from an initializer_list.
  constexpr TupleContainer(std::initializer_list<value_type> init) { assign(init.begin(), init.end()); }

  /// Returns true if the container is empty.
  [[nodiscard]] constexpr bool empty() const noexcept { return std::get<0>(sequences_).empty(); }

  /// Returns the size of the container.
  [[nodiscard]] constexpr size_type size() const noexcept { return std::get<0>(sequences_).size(); }

  /// Returns the capacity of the container.
  [[nodiscard]] constexpr size_type capacity() const noexcept { return std::get<0>(sequences_).capacity(); }

  /// Clears the container.
  constexpr void clear() noexcept {
    std::apply([](auto&&... containers) { (containers.clear(), ...); }, sequences_);
  }

  /// Replaces elements in the container with copies of those in the range [first, last).
  /**
   * The behavior is undefined if either argument is an iterator into *this.
   */
  template <typename I, typename S>
  constexpr void assign(I first, S last) {
    auto range = ranges::make_subrange(first, last);
    static_assert(ranges::input_range<decltype(range)>);
    if constexpr (ranges::sized_range<decltype(range)>) {
      resize(ranges::size(range));
      ranges::copy(range, begin());
    } else {
      // Optimization to copy as much as we can before we start back inserting.
      // Back insertion time is severely punished by having multiple containers.
      //
      // There are two bad cases for this implementation:
      // - The capacity is too large compared to the size of the input range, so
      //   we allocate and default construct unnecessarily.
      // - The capacity is too low compared to the size of the input range, so
      //   there will be a lot of back inserting.
      //
      // If common guidelines of calling reserve with the expected size are followed,
      // then this is quite fast, even when we don't know the exact size of the range
      // in advance.
      resize(capacity());
      // Copy elements until we reach current capacity.
      auto limited_range = range | ranges::views::take(size());
      auto [last_in, last_out] = ranges::copy(limited_range, begin());
      const auto copied_size = static_cast<size_type>(ranges::distance(begin(), last_out));
      if (size() == copied_size) {
        // Back insert the remaining elements if any.
        ranges::copy(last_in.base(), last, ranges::back_inserter(*this));
      } else {
        // Remove extra elements by resizing to the correct size.
        resize(copied_size);
      }
    }
  }

  /// Replaces elements in the container with a copy of each element in range.
  template <typename R>
  constexpr void assign_range(R&& range) {
    assign(ranges::begin(range), ranges::end(range));
  }

  /// Reserves the specified capacity.
  /**
   * If the specified new capacity is greater than the current capacity, new storage is allocated.
   * Otherwise, the method does nothing.
   *
   * \param new_cap New capacity of the container.
   */
  constexpr void reserve(size_type new_cap) {
    std::apply([new_cap](auto&&... containers) { (containers.reserve(new_cap), ...); }, sequences_);
  }

  /// Resizes the container.
  /**
   * The container is resized to have exactly `count` elements.
   * If the specified size is less than the current size, the first `count` elements of the container will be kept.
   * Otherwise, the container is extended with default initialized values.
   *
   * \param count New size of the container.
   */
  constexpr void resize(size_type count) {
    std::apply([count](auto&&... containers) { (containers.resize(count), ...); }, sequences_);
  }

  /// Adds an element at the end of the container.
  /**
   * \param value The element to be appended.
   */
  constexpr void push_back(const value_type& value) {
    push_back_impl(value, std::make_index_sequence<sizeof...(Types)>());
  }

  /// \overload
  constexpr void push_back(value_type&& value) {
    push_back_impl(std::move(value), std::make_index_sequence<sizeof...(Types)>());
  }

  /// Returns an iterator to the first element of the container.
  [[nodiscard]] constexpr auto begin() const { return all().begin(); }

  /// Returns an iterator to the last element of the container.
  [[nodiscard]] constexpr auto end() const { return all().end(); }

  /// \overload
  [[nodiscard]] constexpr auto begin() { return all().begin(); }

  /// \overload
  [[nodiscard]] constexpr auto end() { return all().end(); }

 private:
  std::tuple<InternalContainer<Types>...> sequences_;

  template <typename T, std::size_t... Ids>
  constexpr void push_back_impl(T value, std::index_sequence<Ids...>) {
    (std::get<Ids>(sequences_).push_back(std::get<Ids>(value)), ...);
  }

  [[nodiscard]] constexpr auto all() const {
    return std::apply(
        [](auto&... containers) {  //
          return beluga::views::zip(containers...) | ranges::views::const_;
        },
        sequences_);
  }

  [[nodiscard]] constexpr auto all() {
    return std::apply(
        [](auto&... containers) {  //
          return beluga::views::zip(containers...);
        },
        sequences_);
  }
};

/// Shorthand for a vector with the default allocator.
template <class T>
using Vector = std::vector<T, std::allocator<T>>;

/// Shorthand for a tuple of vectors with the default allocator.
/**
 * This is needed so we can define deduction guides for this type.
 */
template <class T>
class TupleVector : public TupleContainer<Vector, T> {
  /// Inherited constructors.
  using TupleContainer<Vector, T>::TupleContainer;
};

/// Deduction guide to construct from iterators.
template <class I, class S, typename = std::enable_if_t<ranges::input_iterator<I> && ranges::input_iterator<S>>>
TupleVector(I, S) -> TupleVector<std_tuple_decay_t<ranges::iter_value_t<I>>>;

}  // namespace beluga

#endif
