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

/// Implementation detail for a policy closure base object.
struct policy_closure_base {
  /// Short-circuited logical AND operation.
  template <class Left, class Right>
  friend constexpr auto operator&&(policy_closure<Left> left, policy_closure<Right> right) {
    return make_policy_closure([=](const auto&... args) mutable -> bool { return left(args...) && right(args...); });
  }

  /// Non-short-circuited logical AND operation.
  template <class Left, class Right>
  friend constexpr auto operator&(policy_closure<Left> left, policy_closure<Right> right) {
    return make_policy_closure([=](const auto&... args) mutable -> bool {
      const bool first = left(args...);
      const bool second = right(args...);
      return first && second;
    });
  }

  /// Short-circuited logical OR operation.
  template <class Left, class Right>
  friend constexpr auto operator||(policy_closure<Left> left, policy_closure<Right> right) {
    return make_policy_closure([=](const auto&... args) mutable -> bool { return left(args...) || right(args...); });
  }

  /// Non-short-circuited logical OR operation.
  template <class Left, class Right>
  friend constexpr auto operator|(policy_closure<Left> left, policy_closure<Right> right) {
    return make_policy_closure([=](const auto&... args) mutable -> bool {
      const bool first = left(args...);
      const bool second = right(args...);
      return first || second;
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
 * A policy closure should be cheaply copyable and its arguments will always be passed
 * by const-reference.
 */
template <class PolicyFn>
struct policy_closure : public policy_closure_base, public PolicyFn {
  /// Default constructor.
  policy_closure() = default;

  /// Conversion constructor.
  constexpr explicit policy_closure(PolicyFn fn) : PolicyFn(std::move(fn)) {}

  using PolicyFn::operator();

  /// Call operator overload.
  /**
   * If the function object can be called with no arguments, enable this overload that
   * takes any amount of arguments so it can be composed with any other policy.
   */
  template <class... Args>
  constexpr auto operator()(Args...) ->  //
      std::enable_if_t<                  //
          std::is_invocable_r_v<bool, PolicyFn> && !std::is_invocable_r_v<bool, PolicyFn, Args...>,
          bool> {
    return (*this)();
  }
};

}  // namespace policies

/// Type erased policy.
template <class... Args>
using any_policy = std::function<bool(Args...)>;

}  // namespace beluga

#endif
