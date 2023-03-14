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

#include <functional>
#include <type_traits>
#include <variant>

/**
 * \file
 * \brief Implementation of extensions to the standard utility library.
 */

namespace beluga::mixin {

/// \cond filter

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

/// \endcond filter

/// \cond detail

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

/// \endcond

/// Type trait to detect that a given type is a variant.
/**
 * If `T` (after decay) is an instantiation of
 * [std::variant](https://en.cppreference.com/w/cpp/utility/variant),
 * provides a member constant value equal to `true`. For any other type, value is `false`.
 *
 * \tparam T A type to check.
 */
template <typename T>
struct is_variant {
  /// Member constant value.
  static constexpr bool value = detail::is_variant_impl<std::decay_t<T>>::value;  // NOLINT
};

/// Helper variable template to detect that a given type is a variant.
template <typename T>
inline constexpr bool is_variant_v = is_variant<T>::value;  // NOLINT

/// Type trait to detect that a given type is a reference wrapper.
/**
 * If `T` (after decay) is an instantiation of
 * [std::reference_wrapper](https://en.cppreference.com/w/cpp/utility/functional/reference_wrapper),
 * provides a member constant value equal to `true`. For any other type, value is `false`.
 *
 * \tparam T A type to check.
 */
template <typename T>
struct is_reference_wrapper {
  /// Member constant value.
  static constexpr bool value = detail::is_reference_wrapper_impl<std::decay_t<T>>::value;  // NOLINT
};

/// Helper variable template to detect that a given type is a reference wrapper.
template <typename T>
inline constexpr bool is_reference_wrapper_v = is_reference_wrapper<T>::value;  // NOLINT

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
      return static_cast<U&>(value);
    }
  } else {
    if constexpr (is_adding_const) {
      return std::move(std::as_const(value));
    } else {
      return std::forward<U>(value);
    }
  }
}

/// Helper function to unwrap a reference_wrapper or forward the given value.
/**
 * \tparam T The type of the input value.
 * \param value A reference_wrapper value to unwrap or a value to forward.
 * \return The unwrapped reference or the forwarded value.
 */
template <class T>
constexpr decltype(auto) maybe_unwrap(T&& value) noexcept {
  if constexpr (is_reference_wrapper_v<T>) {
    return value.get();
  } else {
    return std::forward<T>(value);
  }
}

/// Helper function to create a variant or forward if the value is already a variant.
/**
 * \tparam T The type of the input value.
 * \param value A value to forward or to create a variant instance.
 * \return A variant value whose type is a `reference_wrapper` to the input value
 * or an existing variant value forwarded.
 */
template <class T>
constexpr decltype(auto) maybe_variant(T&& value) noexcept {
  if constexpr (!is_variant_v<T>) {
    return std::variant<std::reference_wrapper<std::decay_t<T>>>(std::ref(value));
  } else {
    return std::forward<T>(value);
  }
}

/// Applies the visitor to a combination of types from variants, also taking non-variant values.
/**
 * See also https://en.cppreference.com/w/cpp/utility/variant/visit.
 *
 * \tparam Visitor Callable type that can be called with any combination of types from variants.
 * \tparam Args Input argument types for the visitor, can be variant and non-variant types.
 * \param vis A callable that accepts every possible alternative from every variant.
 * \param args List of arguments to pass to the visitor.
 */
template <class Visitor, class... Args>
constexpr decltype(auto) visit_everything(Visitor&& vis, Args&&... args) {
  return std::visit(
      [&](auto&&... args) -> decltype(auto) { return std::forward<Visitor>(vis)(maybe_unwrap(args)...); },
      maybe_variant(std::forward<Args>(args))...);
}

/// Creates a tuple object omiting the element types that don't match the given predicate.
/**
 * See also https://en.cppreference.com/w/cpp/utility/tuple/make_tuple.
 *
 * \tparam Pred A template type trait to be used to filter types from the tuple.
 * \tparam Values The types of the input arguments.
 * \param values Zero or more arguments to construct the tuple from.
 */
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
