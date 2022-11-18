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

#pragma once

#include <tuple>
#include <vector>

#include <range/v3/view/zip.hpp>

#include <beluga/type_traits.h>

namespace beluga {

template <template <class> class InternalContainer, class Tuple>
class TupleContainer;

template <template <class> class InternalContainer, template <class...> class Tuple, class... Types>
class TupleContainer<InternalContainer, Tuple<Types...>> {
 public:
  using value_type = Tuple<Types...>;
  using size_type = std::size_t;

  constexpr TupleContainer() = default;

  explicit constexpr TupleContainer(size_type count) : sequences_{((void)sizeof(Types), count)...} {}

  [[nodiscard]] constexpr bool empty() const noexcept { return std::get<0>(sequences_).empty(); }

  [[nodiscard]] constexpr size_type size() const noexcept { return std::get<0>(sequences_).size(); }

  [[nodiscard]] constexpr size_type capacity() const noexcept { return std::get<0>(sequences_).capacity(); }

  constexpr void clear() noexcept {
    std::apply([](auto&&... containers) { (containers.clear(), ...); }, sequences_);
  }

  constexpr void reserve(size_type new_cap) {
    std::apply([new_cap](auto&&... containers) { (containers.reserve(new_cap), ...); }, sequences_);
  }

  constexpr void resize(size_type count) {
    std::apply([count](auto&&... containers) { (containers.resize(count), ...); }, sequences_);
  }

  constexpr void push_back(value_type&& value) {
    push_back_impl(std::move(value), std::make_index_sequence<sizeof...(Types)>());
  }

  constexpr void push_back(const value_type& value) {
    push_back_impl(value, std::make_index_sequence<sizeof...(Types)>());
  }

  constexpr auto view_all() {
    return std::apply([](auto&&... containers) { return ranges::views::zip(containers...); }, sequences_);
  }

 private:
  std::tuple<InternalContainer<Types>...> sequences_;

  template <typename T, std::size_t... Ids>
  constexpr void push_back_impl(T&& value, std::index_sequence<Ids...>) {
    (std::get<Ids>(sequences_).push_back(std::get<Ids>(std::forward<T>(value))), ...);
  }
};

template <template <class> class InternalContainer, class T>
struct container_traits<TupleContainer<InternalContainer, T>> {
  using type = TupleContainer<InternalContainer, T>;
  using value_type = typename type::value_type;
  using size_type = typename type::size_type;

  template <class U>
  static constexpr auto view_all(U&& container) {
    return container.view_all();
  }
};

template <class T>
using Vector = std::vector<T, std::allocator<T>>;

template <class Tuple>
using TupleVector = TupleContainer<Vector, Tuple>;

}  // namespace beluga
