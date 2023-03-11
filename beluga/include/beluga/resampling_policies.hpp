// Copyright 2022 Ekumen, Inc.
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

#ifndef BELUGA_RESAMPLING_POLICIES_HPP
#define BELUGA_RESAMPLING_POLICIES_HPP

#include <beluga/resampling_policies/resample_interval_policy.hpp>
#include <beluga/resampling_policies/resample_on_motion_policy.hpp>
#include <beluga/resampling_policies/resampling_policies_poller.hpp>
#include <beluga/resampling_policies/selective_resampling_policy.hpp>

/**
 * \file
 * \brief Includes all resampling policy related headers.
 */

/**
 * \page ResamplingPolicyPage Beluga named requirements: ResamplingPolicy
 * Requirements for a resampling policy to be used in a `ParticleFilter`.
 *
 * \section ResamplingPolicyRequirements Requirements
 * A type `T` satisfies the `ResamplingPolicy` requirements if the following is satisfied.
 *
 * Given:
 * - An instance `p` of `T`.
 * - An instance `c` of `C`, where is `C` a particle filter that meets the requirements listed in the policy.
 *
 * Then:
 * - `p.do_resampling(c)` will return `true` if resampling must be done according to the policy, `false` otherwise. \n
 *   This function is called in cascade when multiple policies are installed in the filter, and a given filter's
 *   `do_resampling()` function may not be called if a previously queried policy has already voted `false`
 *   (short-circuit evaluation of policies).
 *
 * \section ResamplingPolicyLinks See also
 * - beluga::ResampleIntervalPolicy
 * - beluga::ResampleOnMotionPolicy
 * - beluga::ResamplingPoliciesPoller
 * - beluga::SelectiveResamplingPolicy
 */

#endif
