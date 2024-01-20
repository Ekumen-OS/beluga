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

#ifndef BELUGA_POLICIES_POLICY_HPP
#define BELUGA_POLICIES_POLICY_HPP

namespace beluga {

namespace policies {

/// Forward declaration of policy closure.
template <class PolicyFn>
struct policy_closure;

}  // namespace policies

namespace detail {

/// \cond

struct make_policy_closure_fn {
  template <class Fn>
  constexpr policies::policy_closure<Fn> operator()(Fn fn) const {
    return policies::policy_closure<Fn>{std::forward<Fn>(fn)};
  }
};

/// \endcond

}  // namespace detail

/// Make policy closure function objects.
inline constexpr detail::make_policy_closure_fn make_policy_closure;

namespace policies {

namespace detail {

/// \cond

template <class Left, class Right, class... Args>
struct policy_invocable_traits {
  static constexpr bool both_args = std::is_invocable_r_v<bool, Left, Args...> &&  //
                                    std::is_invocable_r_v<bool, Right, Args...>;

  static constexpr bool left_args = std::is_invocable_r_v<bool, Left, Args...> &&  //
                                    std::is_invocable_r_v<bool, Right>;

  static constexpr bool right_args = std::is_invocable_r_v<bool, Left> &&  //
                                     std::is_invocable_r_v<bool, Right, Args...>;

  static constexpr bool no_args = std::is_invocable_r_v<bool, Left> &&  //
                                  std::is_invocable_r_v<bool, Right>;
};

/// \endcond

}  // namespace detail

/// Implementation detail for a policy closure base object.
struct policy_closure_base {
  /// Short-circuited logical AND operation.
  template <class Left, class Right>
  friend constexpr auto operator&&(policy_closure<Left> left, policy_closure<Right> right) {
    return make_policy_closure([=](const auto&... args) mutable -> bool {
      using invocable_traits = typename detail::policy_invocable_traits<Left, Right, decltype(args)...>;
      if constexpr (invocable_traits::both_args) {
        return left(args...) && right(args...);
      } else if constexpr (invocable_traits::right_args) {
        return left() && right(args...);
      } else if constexpr (invocable_traits::left_args) {
        return left(args...) && right();
      } else {
        static_assert(invocable_traits::no_args);
        return left() && right();
      }
    });
  }

  /// Non-short-circuited logical AND operation.
  template <class Left, class Right>
  friend constexpr auto operator&(policy_closure<Left> left, policy_closure<Right> right) {
    return make_policy_closure([=](const auto&... args) mutable -> bool {
      using invocable_traits = typename detail::policy_invocable_traits<Left, Right, decltype(args)...>;
      if constexpr (invocable_traits::both_args) {
        const bool first = left(args...);
        const bool second = right(args...);
        return first && second;
      } else if constexpr (invocable_traits::right_args) {
        const bool first = left();
        const bool second = right(args...);
        return first && second;
      } else if constexpr (invocable_traits::left_args) {
        const bool first = left(args...);
        const bool second = right();
        return first && second;
      } else {
        static_assert(invocable_traits::no_args);
        const bool first = left();
        const bool second = right();
        return first && second;
      }
    });
  }

  /// Short-circuited logical OR operation.
  template <class Left, class Right>
  friend constexpr auto operator||(policy_closure<Left> left, policy_closure<Right> right) {
    return make_policy_closure([=](const auto&... args) mutable -> bool {
      using invocable_traits = typename detail::policy_invocable_traits<Left, Right, decltype(args)...>;
      if constexpr (invocable_traits::both_args) {
        return left(args...) || right(args...);
      } else if constexpr (invocable_traits::right_args) {
        return left() || right(args...);
      } else if constexpr (invocable_traits::left_args) {
        return left(args...) || right();
      } else {
        static_assert(invocable_traits::no_args);
        return left() || right();
      }
    });
  }

  /// Non-short-circuited logical OR operation.
  template <class Left, class Right>
  friend constexpr auto operator|(policy_closure<Left> left, policy_closure<Right> right) {
    return make_policy_closure([=](const auto&... args) mutable -> bool {
      using invocable_traits = typename detail::policy_invocable_traits<Left, Right, decltype(args)...>;
      if constexpr (invocable_traits::both_args) {
        const bool first = left(args...);
        const bool second = right(args...);
        return first || second;
      } else if constexpr (invocable_traits::right_args) {
        const bool first = left();
        const bool second = right(args...);
        return first || second;
      } else if constexpr (invocable_traits::left_args) {
        const bool first = left(args...);
        const bool second = right();
        return first || second;
      } else {
        static_assert(invocable_traits::no_args);
        const bool first = left();
        const bool second = right();
        return first || second;
      }
    });
  }

  /// Logical NOT operation.
  template <class Fn>
  friend constexpr auto operator!(policy_closure<Fn> fn) {
    return make_policy_closure([=](const auto&... args) mutable -> bool { return !fn(args...); });
  }
};

/// Policy closure template class.
/**
 * A policy is a declarative lazily-evaluated possibly stateful predicate that can be
 * composed with other predicates using overloaded boolean operators.
 *
 * Two policies can be composed if any of the following conditions is satisfied:
 *
 * 1. Both policies can be evaluated with the same arguments.
 * 2. Either of the policies can be evaluated with no arguments.
 *
 * If the second condition applies, the resulting policy will have to be called with
 * the arguments of the one that does take arguments.
 *
 * They should be cheaply copyable and the arguments will be passed by const-reference.
 */
template <class PolicyFn>
struct policy_closure : public policy_closure_base, public PolicyFn {
  /// Default constructor.
  policy_closure() = default;

  /// Conversion constructor.
  constexpr explicit policy_closure(PolicyFn fn) : PolicyFn(std::move(fn)) {}
};

}  // namespace policies

}  // namespace beluga

#endif
