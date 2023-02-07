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

#ifndef BELUGA_AMCL__EXECUTION_POLICY_HPP_
#define BELUGA_AMCL__EXECUTION_POLICY_HPP_

#include <execution>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace beluga_amcl::execution
{

/**
 * \file
 * \brief Utility to allow dinamically choosing an execution policy.
 */

/// Allows dinamically specifying an execution policy.
enum class Policy {seq, par, par_unseq, unseq};

/// Returns the execution policy enum from its name.
inline auto policy_from_string(std::string_view policy_name)
{
  if (policy_name == "seq") {
    return Policy::seq;
  }
  if (policy_name == "unseq") {
    return Policy::unseq;
  }
  if (policy_name == "par") {
    return Policy::par;
  }
  if (policy_name == "par_unseq") {
    return Policy::par_unseq;
  }
  throw std::invalid_argument{"execution policy must be seq, unseq, par or par_unseq"};
}

/// Runs an algorithm using the specified execution policy object.
template<class F, class ... Args>
auto execute_with_policy(Policy p, F f, Args... args)
{
  switch (p) {
    case Policy::par:
      return f(std::execution::par, args ...);
    case Policy::par_unseq:
      return f(std::execution::par_unseq, args ...);
    case Policy::unseq:
      return f(std::execution::unseq, args ...);
    case Policy::seq:  // fallthrough
    default:
      return f(std::execution::seq, args ...);
  }
}

}  // namespace beluga_amcl::execution

#endif  // BELUGA_AMCL__EXECUTION_POLICY_HPP_
