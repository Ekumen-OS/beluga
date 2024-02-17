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
#include <beluga/containers/circular_array.hpp>
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

/// Composes multiple interfaces into a single interface type.
/**
 * \tparam Interfaces Interfaces to combine.
 */
template <class... Interfaces>
struct compose_interfaces : public Interfaces... {
  /// Virtual destructor.
  /**
   * Makes it so at least one of the interfaces provides a virtual destructor.
   */
  ~compose_interfaces() override = default;
};

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
using LaserLocalizationInterface2d = beluga::compose_interfaces<
    BaseParticleFilterInterface,
    StorageInterface<Sophus::SE2d, beluga::Weight>,
    EstimationInterface2d,
    OdometryMotionModelInterface2d,
    LaserSensorModelInterface2d<Map>,
    CustomExtensionMixinInterface>;

/// \cond

/// Wrapper to convert a motion model class into a mixin.
template <typename Mixin, typename Model>
class MotionMixin : public Mixin {
 public:
  using update_type = std::tuple_element_t<0, typename Model::control_type>;
  using state_type = typename Model::state_type;

  template <typename... Args>
  constexpr explicit MotionMixin(Model model, Args&&... args)
      : Mixin(static_cast<decltype(args)>(args)...), model_(std::move(model)) {}

  template <class Generator>
  [[nodiscard]] state_type apply_motion(const state_type& state, Generator& gen) {
    if (state_sampling_function_.has_value()) {
      return (*state_sampling_function_)(state, gen);
    }
    return state;
  }

  void update_motion(const update_type& pose) { state_sampling_function_.emplace(model_(control_window_ << pose)); }

 private:
  Model model_;
  static constexpr auto kWindowSize = std::tuple_size_v<typename Model::control_type>;
  RollingWindow<update_type, kWindowSize> control_window_;
  using StateSamplingFunction =
      decltype(std::declval<Model>()(std::declval<RollingWindow<update_type, kWindowSize>&>()));
  std::optional<StateSamplingFunction> state_sampling_function_;
};

/// Wrapper to convert a sensor model class into a mixin.
template <typename Mixin, typename Model>
struct SensorMixin : public Mixin, public Model {
  using measurement_type = typename Model::measurement_type;
  using map_type = typename Model::map_type;

  template <typename... Args>
  constexpr explicit SensorMixin(Model model, Args&&... args)
      : Model(std::move(model)), Mixin(static_cast<decltype(args)>(args)...) {}

  void update_sensor(measurement_type&& measurement) { Model::update_sensor(std::move(measurement)); }
  void update_map(map_type&& map) { Model::update_map(std::move(map)); }
};

/// \endcond

/// An implementation of Monte Carlo Localization.
/**
 * An instance of this class template implements beluga::LaserLocalizationInterface2d
 * and \ref LocalizationPage.
 *
 * \tparam MotionModel A class that implements \ref MotionModelPage.
 * \tparam SensorModel A class that implements \ref SensorModelPage.
 * \tparam Map Environment representation type consistent with the sensor descriptor.
 * \tparam Interface An optional mixin interface for the user to customize the filter.
 * \tparam CustomExtensionMixin An optional mixin type for the user to customize the filter.
 */
template <
    class MotionModel,
    class SensorModel,
    class Map,
    class Interface = NullMixinInterface,
    template <class> class CustomExtensionMixin = NullMixin>
using MonteCarloLocalization2d = ciabatta::mixin<
    BootstrapParticleFilter,
    ciabatta::curry<StructureOfArrays, Sophus::SE2d, beluga::Weight>::mixin,
    WeightedStateEstimator2d,
    ciabatta::curry<RandomStateGenerator, Sophus::SE2d, Map>::template mixin,
    NaiveSampler,
    FixedLimiter,
    ciabatta::curry<MotionMixin, MotionModel>::template mixin,
    ciabatta::curry<SensorMixin, SensorModel>::template mixin,
    CustomExtensionMixin,
    ciabatta::provides<Interface>::template mixin>;

/// An implementation of Adaptive Monte Carlo Localization.
/**
 * An instance of this class template implements beluga::LaserLocalizationInterface2d
 * and \ref LocalizationPage.
 *
 * \tparam MotionModel A class that implements \ref MotionModelPage.
 * \tparam SensorModel A class that implements \ref SensorModelPage.
 * \tparam Map Environment representation type consistent with the sensor descriptor.
 * \tparam Interface An optional mixin interface for the user to customize the filter.
 * \tparam CustomExtensionMixin An optional mixin type for the user to customize the filter.
 */
template <
    class MotionModel,
    class SensorModel,
    class Map,
    class Interface = NullMixinInterface,
    template <class> class CustomExtensionMixin = NullMixin>
using AdaptiveMonteCarloLocalization2d = ciabatta::mixin<
    BootstrapParticleFilter,
    ciabatta::curry<StructureOfArrays, Sophus::SE2d, beluga::Weight, beluga::Cluster>::mixin,
    WeightedStateEstimator2d,
    ciabatta::curry<RandomStateGenerator, Sophus::SE2d, Map>::template mixin,
    AdaptiveSampler,
    ciabatta::curry<KldLimiter, Sophus::SE2d>::mixin,
    ciabatta::curry<MotionMixin, MotionModel>::template mixin,
    ciabatta::curry<SensorMixin, SensorModel>::template mixin,
    CustomExtensionMixin,
    ciabatta::provides<Interface>::template mixin>;

}  // namespace beluga

#endif
