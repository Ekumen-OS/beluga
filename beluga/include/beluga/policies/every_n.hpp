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

#ifndef BELUGA_POLICIES_EVERY_N_HPP
#define BELUGA_POLICIES_EVERY_N_HPP

#include <beluga/policies/policy.hpp>

/**
 * \file
 * \brief Defines a policy for triggering an action every N calls.
 */

namespace beluga::policies {

namespace detail {

/// Implementation detail for the every_n_policy.
/**
 * This policy triggers an action every N calls, where N is a specified count.
 */
struct every_n_policy {
 public:
  /// Constructor.
  /**
   * \param count The count specifying when the action should be triggered (every N calls).
   */
  explicit constexpr every_n_policy(std::size_t count) : count_(count) {}

  /// Call operator overload.
  /**
   * \return True if the action should be triggered, false otherwise.
   *
   * Increments the internal counter and returns true if the current count is a multiple of N.
   */
  constexpr bool operator()() {
    current_ = (current_ + 1) % count_;
    return current_ == 0;
  }

 private:
  std::size_t count_{0};    ///< The count specifying when the action should be triggered.
  std::size_t current_{0};  ///< The current count of calls.
};

/// Implementation detail for an every_n_fn object.
struct every_n_fn {
  /// Overload that creates a policy closure.
  constexpr auto operator()(std::size_t count) const { return beluga::make_policy_closure(every_n_policy{count}); }
};

}  // namespace detail

/// Policy that triggers an action every N calls.
/**
 * This policy is designed to be used for scenarios where an action needs to be performed
 * periodically based on a specified count of calls.
 */
inline constexpr detail::every_n_fn every_n;

}  // namespace beluga::policies

#endif
