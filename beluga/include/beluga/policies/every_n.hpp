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

namespace beluga::policies {

namespace detail {

struct every_n_policy {
 public:
  explicit constexpr every_n_policy(std::size_t count) : count_(count) {}

  constexpr bool operator()() {
    current_ = (current_ + 1) % count_;
    return current_ == 0;
  }

 public:
  std::size_t count_{0};
  std::size_t current_{0};
};

struct every_n_fn {
  constexpr auto operator()(std::size_t count) const { return beluga::make_policy_closure(every_n_policy{count}); }
};

}  // namespace detail

inline constexpr detail::every_n_fn every_n;

}  // namespace beluga::policies

#endif
