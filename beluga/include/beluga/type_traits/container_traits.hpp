// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_TYPE_TRAITS_CONTAINER_TRAITS_HPP
#define BELUGA_TYPE_TRAITS_CONTAINER_TRAITS_HPP

#include <type_traits>

#include <range/v3/view/all.hpp>

/**
 * \file
 * \brief Useful traits for containers.
 */

namespace beluga {

/// Useful traits for a [Container](https://en.cppreference.com/w/cpp/named_req/Container).
/**
 * \tparam T Container type.
 */
template <class T>
struct container_traits {
  /// Value type of the container.
  using value_type = typename T::value_type;
  /// Size type of the container.
  using size_type = typename T::size_type;

  /// Returns a view of all elements of the container.
  template <class U>
  static constexpr auto view_all(U&& container) {
    return container | ranges::views::all;
  }
};

namespace views {

/// Returns a view of all elements of the container.
template <class T>
constexpr auto all(T&& container) {
  return container_traits<std::decay_t<T>>::view_all(container);
}

}  // namespace views

}  // namespace beluga

#endif
