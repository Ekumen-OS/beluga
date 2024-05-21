// Copyright 2023-2024 Ekumen, Inc.
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

#ifndef BELUGA_UTILITY_FORWARD_LIKE_HPP
#define BELUGA_UTILITY_FORWARD_LIKE_HPP

namespace beluga {

/// Returns a reference to a value which has similar properties to `T&&`.
/**
 * Implementation taken from https://en.cppreference.com/w/cpp/utility/forward_like
 * since this feature is only available starting with C++23.
 *
 * The program is ill-formed if `T&&` is not a valid type.
 *
 * \tparam T The type from which to take the properties.
 * \tparam U The type of the input value.
 * \param value A value that needs to be forwarded like type `T`.
 * \return A reference to value of the determined type.
 */
template <class T, class U>
[[nodiscard]] constexpr auto&& forward_like(U&& value) noexcept {
  constexpr bool is_adding_const = std::is_const_v<std::remove_reference_t<T>>;  // NOLINT
  if constexpr (std::is_lvalue_reference_v<T&&>) {
    if constexpr (is_adding_const) {
      return std::as_const(value);
    } else {
      return value;
    }
  } else {
    if constexpr (is_adding_const) {
      return std::move(std::as_const(value));
    } else {
      return std::forward<U>(value);
    }
  }
}

}  // namespace beluga

#endif
