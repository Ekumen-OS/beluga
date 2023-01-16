// Copyright 2022 Ekumen, Inc.
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
#include <range/v3/view/const.hpp>
#include <range/v3/view/zip.hpp>

/**
 * \file
 * \brief Implementation of a tuple of containers.
 */

namespace beluga {

/// Specialization for a tuple with no arguments.
template <template <class> class InternalContainer, class Tuple>
class TupleContainer;

/// An implementation of a tuple of containers, with an interface that looks like a container of tuples.
/**
 * i.e. though this is implemented internally as a Tuple<InternalContainer<Types>...>, the interface looks like
 * a InternalContainer<Tuple<Types>...>.
 * It provides the convenience of the second, but when iterating over only one of the elements of the tuple in the
 * container it has better performance (because of cache locality).
 * To that end, use for example `views::all(tuple_container) | views::elements<0>` to iterate over the first element of
 * the tuple in the container.
 *
 * \tparam InternalContainer Container type constructor, e.g. std::vector.
 * \tparam Tuple Tuple type constructor, e.g. std::pair or std::tuple.
 * \tparam ...Types Elements types of the tuple.
 */
template <template <class> class InternalContainer, template <class...> class Tuple, class... Types>
class TupleContainer<InternalContainer, Tuple<Types...>> {
 public:
  /// Value type of the container.
  using value_type = Tuple<Types...>;
  /// Size type of the container.
  using size_type = std::size_t;

  /// Default constructor, will default initialize all containers in the tuple.
  constexpr TupleContainer() = default;

  /// Constructs a container of size count, all values are default initialized.
  /**
   * \param count Size of the container.
   */
  explicit constexpr TupleContainer(size_type count) : sequences_{((void)sizeof(Types), count)...} {}

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
  constexpr void push_back(value_type&& value) {
    push_back_impl(std::move(value), std::make_index_sequence<sizeof...(Types)>());
  }

  /// \overload
  constexpr void push_back(const value_type& value) {
    push_back_impl(value, std::make_index_sequence<sizeof...(Types)>());
  }

  /// Returns a view of all elements of the container.
  constexpr auto view_all() {
    return std::apply([](auto&&... containers) { return ranges::views::zip(containers...); }, sequences_);
  }

  /// \overload
  constexpr auto view_all() const {
    return std::apply([](auto&&... containers) { return ranges::views::zip(containers...); }, sequences_) |
           ranges::views::const_;
  }

 private:
  std::tuple<InternalContainer<Types>...> sequences_;

  template <typename T, std::size_t... Ids>
  constexpr void push_back_impl(T&& value, std::index_sequence<Ids...>) {
    (std::get<Ids>(sequences_).push_back(std::get<Ids>(std::forward<T>(value))), ...);
  }
};

/// Specialization for a TupleContainer<InternalContainer, T>, see also \ref container_traits.hpp.
template <template <class> class InternalContainer, class T>
struct container_traits<TupleContainer<InternalContainer, T>> {
  /// The container type.
  using type = TupleContainer<InternalContainer, T>;
  /// The container value type.
  using value_type = typename type::value_type;
  /// The container size type.
  using size_type = typename type::size_type;

  /// Returns a view of all the elements in the container.
  template <class U>
  static constexpr auto view_all(U&& container) {
    return container.view_all();
  }
};

/// Shorthand for a std::vector<T, std::allocator<T>>.
template <class T>
using Vector = std::vector<T, std::allocator<T>>;

/// Shorthand for a TupleContainer<Vector, Tuple>.
template <class Tuple>
using TupleVector = TupleContainer<Vector, Tuple>;

}  // namespace beluga

#endif
