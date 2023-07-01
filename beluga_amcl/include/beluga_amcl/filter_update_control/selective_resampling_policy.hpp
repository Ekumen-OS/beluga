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

#ifndef BELUGA_AMCL__FILTER_UPDATE_CONTROL__SELECTIVE_RESAMPLING_POLICY_HPP_
#define BELUGA_AMCL__FILTER_UPDATE_CONTROL__SELECTIVE_RESAMPLING_POLICY_HPP_

#include <algorithm>
#include <utility>

#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/view/transform.hpp>

/**
 * \file
 * \brief Implementation of the selective resampling algorithm for resampling.
 */

namespace beluga_amcl {

/// Parameters used to construct a SelectiveResamplingPolicy instance.
struct SelectiveResamplingPolicyParam {
  /// \brief Enable/disable the SelectiveResampling feature.
  bool enabled{false};
};

/// \brief Implementation of the Selective Resampling algorithm for resampling.
/**
 * The algorithm is based in \cite grisetti2007selectiveresampling, according to the description given in
 * \cite tiacheng2015resamplingmethods.
 * */
class SelectiveResamplingPolicy {
 public:
  /// Parameter type that the constructor uses to configure the policy.
  using param_type = SelectiveResamplingPolicyParam;

  /// \brief Constructs a SelectiveResamplingPolicy instance.
  /**
   * \param configuration Policy configuration data.
   */
  explicit SelectiveResamplingPolicy(const param_type& configuration) : configuration_{configuration} {}

  /// \brief Determine whether resampling must be done according to this policy.
  /**
   * \tparam Concrete Type representing the actual implementation of the filter.
   * It must satisfy the \ref BaseParticleFilterPage "BaseParticleFilter" named requirements.
   */
  template <typename Concrete>
  [[nodiscard]] bool do_resampling(Concrete& filter) {
    if (!configuration_.enabled) {
      return true;
    }
    const auto n_eff =
        1. / ranges::accumulate(filter.weights() | ranges::views::transform([](const auto w) { return w * w; }), 0.);
    const auto n = static_cast<double>(std::size(filter.weights()));
    return n_eff < n / 2.;
  }

 private:
  param_type configuration_;
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__FILTER_UPDATE_CONTROL__SELECTIVE_RESAMPLING_POLICY_HPP_
