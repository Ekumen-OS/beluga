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

#ifndef BELUGA_POLICIES_WHILE_HPP
#define BELUGA_POLICIES_WHILE_HPP

#include <beluga/policies/policy.hpp>

namespace beluga::policies {

namespace detail {

/// Implementation detail for a while policy object.
template <bool Value>
struct while_fn {
  /// lvalue overload, keep a reference to the variable.
  constexpr auto operator()(const bool& value) const noexcept {
    return beluga::make_policy_closure([&]() { return value == Value; });
  }

  /// rvalue overload, copy the value internally.
  constexpr auto operator()(bool&& value) const noexcept {
    return beluga::make_policy_closure([=]() { return value == Value; });
  }
};

}  // namespace detail

inline constexpr detail::while_fn<true> while_true;
inline constexpr detail::while_fn<false> while_false;

}  // namespace beluga::policies

#endif
