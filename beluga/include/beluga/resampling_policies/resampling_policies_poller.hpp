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

#ifndef BELUGA_RESAMPLING_POLICIES_RESAMPLING_POLICIES_POLLER_HPP
#define BELUGA_RESAMPLING_POLICIES_RESAMPLING_POLICIES_POLLER_HPP

#include <tuple>

#include <ciabatta/ciabatta.hpp>

/**
 * \file
 * \brief Implementation of a resampling policies poller.
 */

namespace beluga {

/// Resampling policy poller.
/**
 * \tparam Mixin The mixed-in type.
 * \tparam Policies Zero or more resampling policies. They must satisfy the \ref ResamplingPolicyPage "ResamplingPolicy"
 * named requirements.
 */
template <typename Mixin, typename... Policies>
struct ResamplingPoliciesPoller : public Mixin {
  /// Constructs a ResamplingPoliciesPoller instance.
  /**
   * \tparam ...Rest Arguments types for the remaining mixin constructors.
   * \param policy_configs Configuration parameters for each of the installed resampling policies.
   * \param ...rest Arguments that are not used by this part of the mixin, but by others.
   */
  template <typename... Rest>
  explicit ResamplingPoliciesPoller(const typename Policies::param_type&... policy_configs, Rest&&... rest)
      : Mixin(std::forward<Rest>(rest)...), policies_{policy_configs...} {}

  /// Evaluate all configured resampling policies, return true only if no policy votes "no".
  /**
   * Evaluation of the policies is done one by one, in the order in which they are listed in the "Policies" template
   * parameter pack.
   *
   * Evaluation is done with short-circuit evaluation, meaning that as soon as one of the policies votes "false" the
   * remaining policies will not be queried.
   *
   * If no policies have been installed, then the function will default to always returning "true".
   * */
  [[nodiscard]] bool do_resampling_vote() {
    // resampling voters are queried in cascade, with short-circuit evaluation. Order matters!
    return std::apply([this](auto&... policies) { return (policies.do_resampling(this->self()) && ...); }, policies_);
  }

 private:
  std::tuple<Policies...> policies_;
};

}  // namespace beluga

#endif
