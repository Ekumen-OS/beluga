// Copyright 2023-2024 Ekumen, Inc.
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

#ifndef BELUGA_MIXIN_LOCALIZATION_HPP
#define BELUGA_MIXIN_LOCALIZATION_HPP

#include <beluga/algorithm/estimation.hpp>
#include <beluga/mixin.hpp>
#include <beluga/motion.hpp>
#include <beluga/sensor.hpp>
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

/// A null mixin interface. Meant to be the default value for an optional customization point.
class NullMixinInterface {
 public:
  virtual ~NullMixinInterface() = default;
};

/// A null mixin that does nothing. Meant to be the default value for an optional customization point.
/**
 * \tparam Mixin
 */
template <class Mixin>
class NullMixin : public Mixin {
 public:
  /// @brief  Constructor forwarding all the arguments to the next mixin in the cascade.
  /// @tparam ...Args Types of the arguments to be forwarded down the line.
  /// @param ...args Arguments to be forwarded to the constructors down the line.
  template <class... Args>
  explicit NullMixin(Args&&... args) : Mixin(std::forward<Args>(args)...) {}
};

/// Pure abstract class representing an interface for laser-based localization algorithms.
/**
 * \tparam Map Environment representation type.
 * \tparam CustomExtensionMixinInterface An optional mixin interface for the user to customize the filter.
 */
template <class Map, class CustomExtensionMixinInterface = NullMixinInterface>
using LaserLocalizationInterface2d = beluga::mixin::compose_interfaces<
    BaseParticleFilterInterface,
    StorageInterface<Sophus::SE2d, beluga::Weight>,
    EstimationInterface2d,
    OdometryMotionModelInterface2d,
    LaserSensorModelInterface2d<Map>,
    CustomExtensionMixinInterface>;

/// An implementation of Monte Carlo Localization.
/**
 * An instance of this class template implements beluga::LaserLocalizationInterface2d
 * and \ref LocalizationPage.
 *
 * \tparam MotionDescriptor A descriptor of a mixin that implements \ref MotionModelPage.
 * \tparam SensorDescriptor A descriptor of a mixin that implements \ref SensorModelPage.
 * \tparam Map Environment representation type consistent with the sensor descriptor.
 * \tparam CustomExtensionMixinInterface An optional mixin interface for the user to customize the filter.
 * \tparam CustomExtensionMixin An optional mixin type for the user to customize the filter.
 */
template <
    class MotionDescriptor,
    class SensorDescriptor,
    class Map,
    class CustomExtensionMixinInterface = NullMixinInterface,
    template <class> class CustomExtensionMixin = NullMixin>
using MonteCarloLocalization2d = ciabatta::mixin<
    BootstrapParticleFilter,
    ciabatta::curry<StructureOfArrays, Sophus::SE2d, beluga::Weight>::mixin,
    WeightedStateEstimator2d,
    RandomStateGenerator,
    NaiveSampler,
    FixedLimiter,
    MotionDescriptor::template mixin,
    SensorDescriptor::template mixin,
    CustomExtensionMixin,
    ciabatta::provides<LaserLocalizationInterface2d<Map, CustomExtensionMixinInterface>>::template mixin>;

/// An implementation of Adaptive Monte Carlo Localization.
/**
 * An instance of this class template implements beluga::LaserLocalizationInterface2d
 * and \ref LocalizationPage.
 *
 * \tparam MotionDescriptor A descriptor of a mixin that implements \ref MotionModelPage.
 * \tparam SensorDescriptor A descriptor of a mixin that implements \ref SensorModelPage.
 * \tparam Map Environment representation type consistent with the sensor descriptor.
 * \tparam CustomExtensionMixinInterface An optional mixin interface for the user to customize the filter.
 * \tparam CustomExtensionMixin An optional mixin type for the user to customize the filter.
 */
template <
    class MotionDescriptor,
    class SensorDescriptor,
    class Map,
    class CustomExtensionMixinInterface = NullMixinInterface,
    template <class> class CustomExtensionMixin = NullMixin>
using AdaptiveMonteCarloLocalization2d = ciabatta::mixin<
    BootstrapParticleFilter,
    ciabatta::curry<StructureOfArrays, Sophus::SE2d, beluga::Weight, beluga::Cluster>::mixin,
    WeightedStateEstimator2d,
    RandomStateGenerator,
    AdaptiveSampler,
    ciabatta::curry<KldLimiter, Sophus::SE2d>::mixin,
    MotionDescriptor::template mixin,
    SensorDescriptor::template mixin,
    CustomExtensionMixin,
    ciabatta::provides<LaserLocalizationInterface2d<Map, CustomExtensionMixinInterface>>::template mixin>;

}  // namespace beluga

#endif
