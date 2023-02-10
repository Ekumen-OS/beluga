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

#ifndef BELUGA_ALGORITHM_PARTICLE_FILTER_HPP
#define BELUGA_ALGORITHM_PARTICLE_FILTER_HPP

#include <execution>
#include <utility>

#include <beluga/algorithm/sampling.hpp>
#include <beluga/tuple_vector.hpp>
#include <beluga/type_traits.hpp>
#include <ciabatta/ciabatta.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/algorithm/transform.hpp>
#include <range/v3/view/const.hpp>

/**
 * \file
 * \brief Implementation of particle filters.
 *
 * Contains:
 * - A configurable beluga::MCL implementation of Monte Carlo localization.
 * - A configurable beluga::AMCL implementation of adaptive Monte Carlo localization.
 */

/**
 * \page ParticlePage beluga named requirements: Particle
 * What an implementation of a particle in beluga should provide.
 *
 * \section ParticleRequirements Requirements
 * `T` is a `Particle` if given:
 * - An instance `p` of `T`.
 * - A possibly const instance `cp` of `T`.
 *
 * The following is satisfied:
 * - \c particle_traits<T>::state_type is a valid type.
 * - \c particle_traits<T>::state(cp) returns an instance of \c particle_traits<T>::state_type.
 * - Given `s` an instance of \c particle_traits<T>::state_type, \c particle_traits<T>::state(p) = `s`
 *   is a valid expression and assigns the state `s` to the particle `p`. \n
 *   i.e. after the assignment `s` == \c particle_traits<T>::state(p) is `true`.
 * - \c particle_traits<T>::weight_type is a valid arithmetic type (that is, an integral
 *   type or a floating-point type).
 * - \c particle_traits<T>::weight(cp) is a valid expression and the return type is
 *   \c particle_traits<T>::weight_type.
 * - Given `w` an instance of \c particle_traits<T>::weight_type, \c particle_traits<T>::weight(p) = `w`
 *   is a valid expression and assigns the weight `w` to the particle `p`. \n
 *   i.e. after the assignment `w` == \c particle_traits<T>::weight(p) is `true`.
 */

/**
 * \page ParticleContainerPage beluga named requirements: ParticleContainer
 * What an implementation of a particle container in beluga should provide.
 *
 * \section ParticleContainerRequirements Requirements
 * `T` is a `ParticleContainer` if:
 * - `T` satisfies the [Container](https://en.cppreference.com/w/cpp/named_req/Container)
 *   c++ named requirements.
 * - `T::value_type` satisfies the beluga named requirements \ref ParticlePage "Particle".
 */

/**
 * \page BaseParticleFilterPage beluga named requirements: BaseParticleFilter
 * Base requirements of a particle filter in beluga.
 *
 * \section BaseParticleFilterRequirements Requirements
 * `B` is a `BaseParticleFilter` if given
 * - `T`, the type named by `B::particle_type`.
 * - `S`, the type named by `T::state_type`.
 * - `V`, a range view whose elements are of the same type as `S`.
 * - `p`, a value of type `B`.
 * - `x`, possibly const value of type `V`.
 *
 * The following is satisfied:
 * - `p.particles()` is valid and returns a view to a container that satisfies the
 *   \ref ParticleContainerPage "ParticleContainer" requirements.
 * - `p.sample()` updates the particle filter particles based on the last motion update.
 * - `p.importance_sample()` updates the particle filter particles weight.
 * - `p.resample()` updates the particle filter, generating new particles from the old ones
 *   based on their importance weights.
 * - `p.update()` shorthand for executing the above three steps.
 * - `p.reinitialize(x)` is valid and reinitializes the particles with the given range view values.
 */

/**
 * \page ParticleFilterPage beluga named requirements: ParticleFilter
 * What an implementation of a particle filter in beluga should provide.
 * This is satisfied for example by `beluga::MCL<U, M, S, C>` and `beluga::AMCL<U, M, S, C>`,
 * for any valid `U`, `M`, `S`, `C` (see respective docs for detail).
 *
 * \section ParticleFilterRequirements Requirements
 * `T` is a `ParticleFilter` if:
 * - `T` satisfies the \ref BaseParticleFilterPage "BaseParticleFilter" named requirements.
 * - `T` satisfies the \ref SensorModelPage "SensorModel" named requirements.
 * - `T` satisfies the \ref MotionModelPage "MotionModel" named requirements.
 */
namespace beluga {

/// Base implementation of a particle filter.
/**
 * BootstrapParticleFilter<Mixin, Container> is an implementation of the
 * \ref BaseParticleFilterPage "BaseParticleFilter" named requirements.
 *
 * Given `T`, the type named by \c Container::value_type,
 * - \c particle_traits<T>::state_type must be the same as the `state_type` of the sensor
 *   and motion model.
 *
 * - \c particle_traits<T>::weight_type must be the same as the `weight_type` used in the
 *   sensor mode.
 *
 * \tparam Mixin The mixed-in type. An instance m of Mixin must provide a protected method,
 *  `m.self()`. The return type of `m.self()` must satisfy:
 * - \ref ParticleResamplingPage "ParticleResampling"
 * - \ref ParticleBaselineGenerationPage "ParticleBaselineGeneration"
 * - \ref ParticleSampledGenerationPage "ParticleSampledGeneration"
 * - \ref MotionModelPage "MotionModel"
 * - \ref SensorModelPage "SensorModel"
 *
 * \tparam Container The particle container type.
 *  It must satisfy the \ref ParticleContainerPage "ParticleContainer" named requirements.
 */
template <class Mixin, class Container>
struct BootstrapParticleFilter : public Mixin {
 public:
  /// The particle type.
  using particle_type = typename Container::value_type;

  /// Constructs a BootstrapParticleFilter.
  /**
   * The initial particles are generated using the \ref ParticleBaselineGenerationPage "ParticleBaselineGeneration"
   * implementation of `Mixin`.
   *
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit BootstrapParticleFilter(Args&&... args)
      : Mixin(std::forward<Args>(args)...),
        particles_{initialize_container(
            this->self().template generate_samples<particle_type>() | this->self().take_samples())} {}

  /// Returns a view of the particles container.
  auto particles() { return views::all(particles_); }
  /// Returns a const view of the particles container.
  auto particles() const { return views::all(particles_) | ranges::views::const_; }

  /// Returns a view of the particles states.
  auto states() { return views::states(particles_); }
  /// Returns a const view of the particles states.
  auto states() const { return views::states(particles_) | ranges::views::const_; }

  /// Returns a view of the particles weight.
  auto weights() { return views::weights(particles_); }
  /// Returns a const view of the particles weight.
  auto weights() const { return views::weights(particles_) | ranges::views::const_; }

  /// Reinitializes the filter with a given range view to the new particle states.
  /**
   * The particle filter will be initialized with the first `max_samples` states of the range,
   * determined by the \ref ParticleResamplingPage "ParticleResampling" named requirements.
   *
   * \tparam View Range view type whose elements are of the same type as `particle_type::state_type`.
   * \param states A state range view to reinitialize the filter.
   */
  template <class View>
  void reinitialize(View states) {
    particles_ = initialize_container(states | ranges::views::transform(beluga::make_from_state<particle_type>));
  }

  /// Runs all the sampling steps.
  /**
   * That means, it will sample, importance sample, and finally resample.
   */
  void update() {
    sample();
    importance_sample();
    resample();
  }

  /// Update the particles states based on the motion model and the last pose update.
  /**
   * The update will be done based on the `Mixin` implementation of the
   * \ref MotionModelPage "MotionModel" named requirement.
   */
  void sample() { this->sample(std::execution::par); }

  /// Update the particles states based on the motion model and the last pose update.
  /**
   * Same as the overload that takes no arguments, but it allows specifying a different execution policy.
   * The default is std::execution::par.
   */
  template <typename ExecutionPolicy>
  void sample(ExecutionPolicy&& policy) {
    const auto states = views::states(particles_);
    std::transform(
        std::forward<ExecutionPolicy>(policy), std::begin(states), std::end(states), std::begin(states),
        [this](const auto& state) { return this->self().apply_motion(state); });
  }

  /// Update the particle weights based on the sensor model.
  /**
   * The update will be done based on the `Mixin` implementation of the \ref SensorModelPage "SensorModel"
   * named requirement.
   */
  void importance_sample() { this->importance_sample(std::execution::par); }

  /// Update the particle weights based on the sensor model.
  /**
   * Same as the overload that takes no arguments, but it allows specifying a different execution policy.
   * The default is std::execution::par.
   */
  template <typename ExecutionPolicy>
  void importance_sample(ExecutionPolicy&& policy) {
    const auto states = views::states(particles_);
    std::transform(
        std::forward<ExecutionPolicy>(policy), std::begin(states), std::end(states),
        std::begin(views::weights(particles_)),
        [this](const auto& state) { return this->self().importance_weight(state); });
  }

  /// Resample particles based on ther weights and the resampling policy used.
  /**
   * The update will be done based on the Mixin implementation of the
   * \ref ParticleSampledGenerationPage "ParticleSampledGeneration" and
   * \ref ParticleResamplingPage "ParticleResampling" named requirements.
   */
  void resample() {
    particles_ = initialize_container(this->self().generate_samples_from(particles_) | this->self().take_samples());
  }

 private:
  Container particles_;

  /// Initializes a new particle container from an input view.
  /**
   * The size of the container will be at most `max_samples`, determined by the ParticleResampling
   * named requirements.
   *
   * \tparam View Range view type whose elements are of the same type as `Container::value_type`.
   * \param input A particle range view to initialize the container.
   */
  template <class View>
  [[nodiscard]] Container initialize_container(View input) const {
    const std::size_t size = this->self().max_samples();
    auto container = Container{size};
    const auto first = std::begin(views::all(container));
    const auto last = ranges::copy(input | ranges::views::take(size), first).out;
    container.resize(std::distance(first, last));
    return container;
  }
};

/// An implementation of Monte Carlo localization.
/**
 * MCL<U, M, S, C> is an implementation of the \ref ParticleFilterPage "ParticleFilter"
 * named requirements.
 *
 * \tparam MotionModel MotionModel<AMCL> must implement the \ref MotionModelPage "MotionModel" named requirement.
 * \tparam SensorModel MotionModel<AMCL> must implement the \ref SensorModelPage "SensorModel" named requirement.
 * \tparam State The state of the particle type.
 * \tparam Container The particle container type.
 *  It must satisfy the \ref ParticleContainerPage "ParticleContainer" named requirements.
 */
template <
    template <class>
    class MotionModel,
    template <class>
    class SensorModel,
    class State,
    class Container = TupleVector<std::pair<State, double>>>
struct MCL : public ciabatta::mixin<
                 MCL<MotionModel, SensorModel, State, Container>,
                 ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
                 ciabatta::curry<BaselineGeneration>::template mixin,
                 ciabatta::curry<NaiveGeneration>::template mixin,
                 ciabatta::curry<FixedResampling>::template mixin,
                 MotionModel,
                 SensorModel> {
  using ciabatta::mixin<
      MCL<MotionModel, SensorModel, State, Container>,
      ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
      ciabatta::curry<BaselineGeneration>::template mixin,
      ciabatta::curry<NaiveGeneration>::template mixin,
      ciabatta::curry<FixedResampling>::template mixin,
      MotionModel,
      SensorModel>::mixin;
};

/// An implementation of adaptive Monte Carlo localization.
/**
 * AMCL<U, M, S, C> is an implementation of the \ref ParticleFilterPage "ParticleFilter"
 * named requirements.
 *
 * \tparam MotionModel MotionModel<AMCL> must implement the \ref MotionModelPage "MotionModel" named requirement.
 * \tparam SensorModel MotionModel<AMCL> must implement the \ref SensorModelPage "SensorModel" named requirement.
 * \tparam State The state of the particle type.
 * \tparam Container The particle container type.
 *  It must satisfy the \ref ParticleContainerPage "ParticleContainer" named requirements.
 */
template <
    template <class>
    class MotionModel,
    template <class>
    class SensorModel,
    class State,
    class Container = TupleVector<std::tuple<State, double, std::size_t>>>
struct AMCL : public ciabatta::mixin<
                  AMCL<MotionModel, SensorModel, State, Container>,
                  ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
                  ciabatta::curry<BaselineGeneration>::template mixin,
                  ciabatta::curry<AdaptiveGeneration>::template mixin,
                  ciabatta::curry<KldResampling>::template mixin,
                  MotionModel,
                  SensorModel> {
  using ciabatta::mixin<
      AMCL<MotionModel, SensorModel, State, Container>,
      ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
      ciabatta::curry<BaselineGeneration>::template mixin,
      ciabatta::curry<AdaptiveGeneration>::template mixin,
      ciabatta::curry<KldResampling>::template mixin,
      MotionModel,
      SensorModel>::mixin;
};

}  // namespace beluga

#endif
