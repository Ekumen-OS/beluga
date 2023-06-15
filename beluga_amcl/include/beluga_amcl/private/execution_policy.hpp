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

#ifndef BELUGA_AMCL__PRIVATE__EXECUTION_POLICY_HPP_
#define BELUGA_AMCL__PRIVATE__EXECUTION_POLICY_HPP_

#include <execution>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace beluga_amcl::execution {

/**
 * \file
 * \brief Utility to allow dinamically choosing an execution policy.
 */

/// Allows dinamically specifying an execution policy.
using Policy = std::variant<std::execution::sequenced_policy, std::execution::parallel_policy>;

/// Returns the execution policy from its name.
inline Policy policy_from_string(std::string_view policy_name) {
  if (policy_name == "seq") {
    return std::execution::seq;
  }
  if (policy_name == "par") {
    return std::execution::par;
  }
  throw std::invalid_argument{"execution policy must be seq, par"};
}

}  // namespace beluga_amcl::execution

#endif  // BELUGA_AMCL__PRIVATE__EXECUTION_POLICY_HPP_
