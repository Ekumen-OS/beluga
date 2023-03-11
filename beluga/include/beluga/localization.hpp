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
 */

namespace beluga {

/**
 * \page LocalizationPage Beluga named requirements: Localization
 * What an implementation of a localization algorithm in Beluga should provide.
 *
 * \section LocalizationRequirements Requirements
 * `T` is a `Localization` type if:
 * - `T` satisfies \ref BaseParticleFilterPage.
 * - `T` satisfies \ref StoragePolicyPage.
 * - `T` satisfies \ref StateEstimatorPage.
 * - `T` satisfies \ref SensorModelPage.
 * - `T` satisfies \ref MotionModelPage.
 *
 * \section LocalizationLinks See also
 * - \ref localization.hpp
 */

/// Pure abstract class representing an interface for laser-based localization algorithms.
using LaserLocalizationInterface2d = beluga::mixin::compose_interfaces<
    BaseParticleFilterInterface,
    StorageInterface<Sophus::SE2d, double>,
    EstimationInterface2d,
    OdometryMotionModelInterface2d,
    LaserSensorModelInterface2d>;

/// Commonly used resampling policies combined in a single mixin.
using CombinedResamplingPolicy = ciabatta::
    curry<ResamplingPoliciesPoller, ResampleOnMotionPolicy, ResampleIntervalPolicy, SelectiveResamplingPolicy>;

/// An implementation of Monte Carlo Localization.
/**
 * An instance of this class template implements beluga::LaserLocalizationInterface2d
 * and \ref LocalizationPage.
 *
 * \tparam MotionDescriptor A descriptor of a mixin that implements \ref MotionModelPage.
 * \tparam SensorDescriptor A descriptor of a mixin that implements \ref SensorModelPage.
 */
template <class MotionDescriptor, class SensorDescriptor>
using MonteCarloLocalization2d = ciabatta::mixin<
    BootstrapParticleFilter,
    ciabatta::curry<StructureOfArrays, Sophus::SE2d, double>::mixin,
    SimpleStateEstimator2d,
    RandomStateGenerator,
    NaiveSampler,
    FixedLimiter,
    CombinedResamplingPolicy::template mixin,
    MotionDescriptor::template mixin,
    SensorDescriptor::template mixin,
    ciabatta::provides<LaserLocalizationInterface2d>::mixin>;

/// An implementation of Adaptive Monte Carlo Localization.
/**
 * An instance of this class template implements beluga::LaserLocalizationInterface2d
 * and \ref LocalizationPage.
 *
 * \tparam MotionDescriptor A descriptor of a mixin that implements \ref MotionModelPage.
 * \tparam SensorDescriptor A descriptor of a mixin that implements \ref SensorModelPage.
 */
template <class MotionDescriptor, class SensorDescriptor>
using AdaptiveMonteCarloLocalization2d = ciabatta::mixin<
    BootstrapParticleFilter,
    ciabatta::curry<StructureOfArrays, Sophus::SE2d, double, std::size_t>::mixin,
    SimpleStateEstimator2d,
    RandomStateGenerator,
    AdaptiveSampler,
    KldLimiter,
    CombinedResamplingPolicy::template mixin,
    MotionDescriptor::template mixin,
    SensorDescriptor::template mixin,
    ciabatta::provides<LaserLocalizationInterface2d>::mixin>;

}  // namespace beluga

#endif
