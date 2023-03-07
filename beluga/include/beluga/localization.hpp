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

#ifndef BELUGA_LOCALIZATION_HPP
#define BELUGA_LOCALIZATION_HPP

#include <beluga/algorithm/estimation.hpp>
#include <beluga/algorithm/particle_filter.hpp>
#include <beluga/algorithm/sampling.hpp>
#include <beluga/mixin.hpp>
#include <beluga/motion.hpp>
#include <beluga/resampling_policies/resample_interval_policy.hpp>
#include <beluga/resampling_policies/resample_on_motion_policy.hpp>
#include <beluga/resampling_policies/resampling_policies_poller.hpp>
#include <beluga/resampling_policies/selective_resampling_policy.hpp>
#include <beluga/sensor.hpp>
#include <beluga/storage.hpp>
#include <ciabatta/ciabatta.hpp>

/**
 * \file
 * \brief Implementation of localization algorithms.
 *
 * Contains:
 * - A configurable beluga::MonteCarloLocalization implementation of Monte Carlo Localization.
 * - A configurable beluga::AdaptiveMonteCarloLocalization implementation of Adaptive Monte Carlo Localization.
 */

namespace beluga {

using LaserLocalizationInterface2d = beluga::mixin::compose_interfaces<
    BaseParticleFilterInterface,
    StorageInterface<Sophus::SE2d, double>,
    EstimationInterface2d,
    OdometryMotionModelInterface2d,
    LaserSensorModelInterface2d>;

/// An implementation of Monte Carlo Localization.
/**
 * MCL<U, M> is an implementation of the \ref ParticleFilterPage "ParticleFilter"
 * named requirements.
 *
 * \tparam MotionModel MotionModel<Mixin> must implement the \ref MotionModelPage "MotionModel" named requirement.
 * \tparam SensorModel SensorModel<Mixin> must implement the \ref SensorModelPage "SensorModel" named requirement.
 */
template <template <class> class MotionModel, template <class> class SensorModel>
using MonteCarloLocalization2d = ciabatta::mixin<
    BootstrapParticleFilter,
    ciabatta::curry<StructureOfArrays, Sophus::SE2d, double>::mixin,
    SimpleStateEstimator2d,
    RandomStateGenerator,
    NaiveSampler,
    FixedLimiter,
    ciabatta::
        curry<ResamplingPoliciesPoller, ResampleOnMotionPolicy, ResampleIntervalPolicy, SelectiveResamplingPolicy>::
            mixin,
    MotionModel,
    SensorModel,
    ciabatta::provides<LaserLocalizationInterface2d>::mixin>;

/// An implementation of Adaptive Monte Carlo Localization.
/**
 * AMCL<U, M> is an implementation of the \ref ParticleFilterPage "ParticleFilter"
 * named requirements.
 *
 * \tparam MotionModel MotionModel<Mixin> must implement the \ref MotionModelPage "MotionModel" named requirement.
 * \tparam SensorModel SensorModel<Mixin> must implement the \ref SensorModelPage "SensorModel" named requirement.
 */
template <template <class> class MotionModel, template <class> class SensorModel>
using AdaptiveMonteCarloLocalization2d = ciabatta::mixin<
    BootstrapParticleFilter,
    ciabatta::curry<StructureOfArrays, Sophus::SE2d, double, std::size_t>::mixin,
    SimpleStateEstimator2d,
    RandomStateGenerator,
    AdaptiveSampler,
    KldLimiter,
    ciabatta::
        curry<ResamplingPoliciesPoller, ResampleOnMotionPolicy, ResampleIntervalPolicy, SelectiveResamplingPolicy>::
            mixin,
    MotionModel,
    SensorModel,
    ciabatta::provides<LaserLocalizationInterface2d>::mixin>;

}  // namespace beluga

#endif
