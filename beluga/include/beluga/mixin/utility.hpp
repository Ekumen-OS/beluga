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

#ifndef BELUGA_MIXIN_UTILITY_HPP
#define BELUGA_MIXIN_UTILITY_HPP

#include <type_traits>
#include <variant>

namespace beluga::mixin {

template <typename...>
struct list;

template <typename... Lists>
struct concat;

template <template <typename> class Pred, typename T>
using filter_helper = std::conditional_t<Pred<T>::value, list<T>, list<>>;

template <template <typename> class Pred, typename... Ts>
using filter = typename concat<filter_helper<Pred, Ts>...>::type;

template <>
struct concat<> {
  using type = list<>;
};
template <typename... Ts>
struct concat<list<Ts...>> {
  using type = list<Ts...>;
};
template <typename... Ts, typename... Us>
struct concat<list<Ts...>, list<Us...>> {
  using type = list<Ts..., Us...>;
};
template <typename... Ts, typename... Us, typename... Rest>
struct concat<list<Ts...>, list<Us...>, Rest...> {
  using type = typename concat<list<Ts..., Us...>, Rest...>::type;
};

namespace detail {

template <typename T>
struct is_variant_impl : std::false_type {};

template <typename... Args>
struct is_variant_impl<std::variant<Args...>> : std::true_type {};

template <typename T>
struct is_reference_wrapper_impl : std::false_type {};

template <typename T>
struct is_reference_wrapper_impl<std::reference_wrapper<T>> : std::true_type {};

}  // namespace detail

template <typename T>
struct is_variant {
  static constexpr bool value = detail::is_variant_impl<std::decay_t<T>>::value;  // NOLINT
};

template <typename T>
inline constexpr bool is_variant_v = is_variant<T>::value;  // NOLINT

template <typename T>
struct is_reference_wrapper {
  static constexpr bool value = detail::is_reference_wrapper_impl<std::decay_t<T>>::value;  // NOLINT
};

template <typename T>
inline constexpr bool is_reference_wrapper_v = is_reference_wrapper<T>::value;  // NOLINT

template <class T, class U>
[[nodiscard]] constexpr auto&& forward_like(U&& x) noexcept {
  constexpr bool is_adding_const = std::is_const_v<std::remove_reference_t<T>>;  // NOLINT
  if constexpr (std::is_lvalue_reference_v<T&&>) {
    if constexpr (is_adding_const) {
      return std::as_const(x);
    } else {
      return static_cast<U&>(x);
    }
  } else {
    if constexpr (is_adding_const) {
      return std::move(std::as_const(x));
    } else {
      return std::forward<U>(x);
    }
  }
}

template <class T>
constexpr decltype(auto) maybe_unwrap(T&& value) noexcept {
  if constexpr (is_reference_wrapper_v<T>) {
    return value.get();
  } else {
    return std::forward<T>(value);
  }
}

template <class T>
constexpr decltype(auto) maybe_variant(T&& value) noexcept {
  if constexpr (!is_variant_v<T>) {
    return std::variant<std::reference_wrapper<std::decay_t<T>>>(std::ref(value));
  } else {
    return std::forward<T>(value);
  }
}

template <class Visitor, class... Args>
constexpr decltype(auto) visit_everything(Visitor&& vis, Args&&... args) {
  return std::visit(
      [&](auto&&... args) { return std::forward<Visitor>(vis)(maybe_unwrap(args)...); },
      maybe_variant(std::forward<Args>(args))...);
}

template <template <typename> class Pred, class... Values>
constexpr auto make_tuple_with(Values&&... values) {
  constexpr auto helper = [](auto&& value) {  // NOLINT
    if constexpr (Pred<decltype(value)>::value) {
      return std::forward_as_tuple(std::forward<decltype(value)>(value));
    } else {
      return std::tuple<>{};
    }
  };
  return std::tuple_cat(helper(std::forward<decltype(values)>(values))...);
}

}  // namespace beluga::mixin

#endif
