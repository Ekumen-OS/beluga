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

#ifndef BELUGA_RESAMPLING_POLICIES_RESAMPLE_INTERVAL_POLICY_HPP
#define BELUGA_RESAMPLING_POLICIES_RESAMPLE_INTERVAL_POLICY_HPP

#include <optional>

/**
 * \file
 * \brief Implementation of the resample interval algorithm for resampling.
 */

namespace beluga {

/// Parameters used to construct a ResampleIntervalPolicy instance.
struct ResampleIntervalPolicyParam {
  /// Interval of calls to do_resampling() out of which only the last iteration will do resampling.
  std::size_t resample_interval_count{1};
};

/// Implementation of the Resample Interval algorithm for resampling.
/**
 * ResampleIntervalPolicy is an implementation of the \ref ResamplingPolicyPage "ResamplingPolicy" named requirements.
 * */
struct ResampleIntervalPolicy {
 public:
  /// Parameter type that the constructor uses to configure the policy.
  using param_type = ResampleIntervalPolicyParam;

  /// Constructs a ResampleIntervalPolicy instance.
  /**
   * \param configuration Policy configuration data.
   */
  explicit ResampleIntervalPolicy(const param_type& configuration) : configuration_{configuration} {}

  /// Vote whether resampling must be done according to this policy.
  /**
   * \tparam Concrete Type representing the concrete implementation of the filter.
   */
  template <typename Concrete>
  [[nodiscard]] bool do_resampling([[maybe_unused]] Concrete& filter) {
    filter_update_counter_ = (filter_update_counter_ + 1) % configuration_.resample_interval_count;
    return (filter_update_counter_ == 0);
  }

 private:
  param_type configuration_;              //< Policy configuration
  std::size_t filter_update_counter_{0};  //< Current cycle phase
};

}  // namespace beluga

#endif
