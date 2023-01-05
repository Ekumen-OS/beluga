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

#include <beluga/algorithm/estimation.hpp>
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
 * - A configurable beluga::MCL implementation of montecarlo localization.
 * - A configurable beluga::AMCL implementation of adaptive montecarlo localization.
 */

// TODO(ivanpauno): Maybe just use one page for all requirements, and use sections and subsections....
/**
 * \page ParticleRequirements beluga named requirements: Particle.
 * What an implementation of a particle in beluga should provide.
 *
 * \section Requirements
 * T is a Particle if, for a given instance p of T:
 * - ::beluga::particle_traits::state_type<T> is a valid type.
 * - ::beluga::state(p) is a valid expression and an instance of
 *   ::beluga::particle_traits::state_type<T>.
 * - ::beluga::weigth(p) is a valid expression, the return type must be a scalar (e.g. double).
 * <--TODO(ivanpauno): Add particle_traits<T>::weight_type?? (or scalar_type?)-->
 * <--TODO(ivanpauno): Add particle_traits<T>::covariance_type??, or maybe that should be specified by State.-->
 */

/**
 * \page ParticleContainerRequirements beluga named requirements: ParticleContainer.
 * What an implementation of a particle container in beluga should provide.
 *
 * \section Requirements
 * T is a ParticleContainer if:
 * - T satisfies the [Container](https://en.cppreference.com/w/cpp/named_req/Container)
 *   c++ named requirements.
 * - T::value_type satisfies the beluga named requirements \ref ParticleRequirements "Particle".
 */

/**
 * \page ParticleFilterRequirements beluga named requirements: ParticleFilter.
 * What an implementation of a particle filter in beluga should provide.
 * This is satisfied for example by beluga::MCL<U, M, S, C> and beluga::AMCL<U, M, S, C>.
 * for any valid U, M, S, C (see respective docs for detail).
 *
 * \section Requirements
 * T is a ParticleFilter if given an instance of T p, the following is satisfied:
 * <!--TODO(ivanpauno): Add named requirements links when documented-->
 * - T also satisfies the SensorModel named requirements. <!--update_sensor()-->
 *   <!--
 *   I'm not sure about likelihood_field() ...
 *   Maybe something that gives a occupancy grid from the sensor model is fine.
 *   Even if the sensor model is implemented in another way e.g. NDT transform,
 *   a conversion to an occupancy grid should be possible.
 *   -->
 * - T also satisfies the MotionModel named requirements.  <!--update_motion() and last_pose()-->
 * - T also satisfies the MotionModel named requirements.
 * - T also satisfies the StateEstimation named requirements.
 * - p.particles() is valid and returns a type that satisfies the
 *   \ref ParticleContainerRequirements "ParticleContainer" requirements.
 * - p.sample() updates the particle filter particles based on the last motion update.
 * - p.importance_sample() updates the particle filter particles weight.
 * - p.resample() updates the particle filter, generating new particles from the old ones
 *   based on their importance weights.
 * - p.update() shorthand for executing the above three steps.
 *
 * <!--TODO(ivanpauno):
 * Minor API discussion:
 *
 * - Should p.update_motion() be called p.update_last_pose() and p.sample() called p.update_motion()?
 *   Or maybe p.update_particles_using_motion_model()?
 * -->
 */
namespace beluga {


/// Base implementation of a particle filter.
/**
 * BootstrapParticleFilter<Mixin, Container> is an implementation of the
 * \ref ParticleFilterRequirements "ParticleFilter" named requirements, except it
 * does not satisfy the StateEstimator requirement. <!--TODO(ivanpauno): add link-->
 *
 * \tparam Mixin must also satisfy the named requirements:
 * <!--TODO(ivanpauno): Add links when documented.-->
 * - ParticleResampling <!--for max_samples(), take_samples()-->
 * - ParticleBaselineGeneration <!--for generate_samples()-->
 * - ParticleSampledGeneration <!--for generate_samples_from()-->
 * - MotionModel <!-- for apply_motion()-->
 * - SensorModel <!-- for importance_weight()-->
 * \tparam Container The particle container type.
 *  It must satisfy the \ref ParticleContainerRequirements "ParticleContainer" named requirements.
 */
template <class Mixin, class Container>
struct BootstrapParticleFilter : public Mixin {
 public:
  /// Constructs a BootstrapParticleFilter.
  /**
   * The initial particles are generated using the ParticleBaselineGeneration implementation of
   * Mixin.
   *
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit BootstrapParticleFilter(Args&&... args)
      : Mixin(std::forward<Args>(args)...), particles_{this->self().max_samples()} {
    using particle_type = typename Container::value_type;
    auto first = std::begin(views::all(particles_));
    auto last =
        ranges::copy(this->self().template generate_samples<particle_type>() | this->self().take_samples(), first).out;
    particles_.resize(std::distance(first, last));
  }

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
   * The update will be done based on the Mixin implementation of the MotionModel named
   * requirement.
   */
  void sample() {
    const auto states = views::states(particles_);
    std::transform(
        std::execution::par, std::begin(states), std::end(states), std::begin(states),
        [this](const auto& state) { return this->self().apply_motion(state); });
  }

  /// Update the particle weights based on the sensor model.
  /**
   * The update will be done based on the Mixin implementation of the SensorModel named
   * requirement.
   */
  void importance_sample() {
    const auto states = views::states(particles_);
    std::transform(
        std::execution::par, std::begin(states), std::end(states), std::begin(views::weights(particles_)),
        [this](const auto& state) { return this->self().importance_weight(state); });
  }

  /// Resample particles based on ther weights and the resampling policy used.
  /**
   * The update will be done based on the Mixin implementation of the ParticleSampledGeneration and
   * ParticleResampling named requirements.
   */
  void resample() {
    auto new_particles = Container{this->self().max_samples()};
    auto first = std::begin(views::all(new_particles));
    auto last = ranges::copy(this->self().generate_samples_from(particles_) | this->self().take_samples(), first).out;
    new_particles.resize(std::distance(first, last));
    particles_ = std::move(new_particles);
  }

 private:
  Container particles_;
};

/// An implementation of monte carlo localization.
/**
 * MCL<U, M, S, C> is an implementation of the \ref ParticleFilterRequirements "ParticleFilter"
 * named requirements.
 *
 * \tparam MotionModel MotionModel<MCL> must implement the MotionModel named requirement.
 * \tparam SensorModel MotionModel<MCL> must implement the SensorModel named requirement.
 * \tparam State The state of the particle type.
 * \tparam Container The particle container type.
 *  It must satisfy the \ref ParticleContainerRequirements "ParticleContainer" named requirements.
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
                 ciabatta::curry<SimpleEstimation>::template mixin,
                 MotionModel,
                 SensorModel> {
  using ciabatta::mixin<
      MCL<MotionModel, SensorModel, State, Container>,
      ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
      ciabatta::curry<BaselineGeneration>::template mixin,
      ciabatta::curry<NaiveGeneration>::template mixin,
      ciabatta::curry<FixedResampling>::template mixin,
      ciabatta::curry<SimpleEstimation>::template mixin,
      MotionModel,
      SensorModel>::mixin;
};

/// An implementation of adaptive monte carlo localization.
/**
 * AMCL<U, M, S, C> is an implementation of the \ref ParticleFilterRequirements "ParticleFilter"
 * named requirements.
 *
 * \tparam MotionModel MotionModel<AMCL> must implement the MotionModel named requirement.
 * \tparam SensorModel MotionModel<AMCL> must implement the SensorModel named requirement.
 * \tparam State The state of the particle type.
 * \tparam Container The particle container type.
 *  It must satisfy the \ref ParticleContainerRequirements "ParticleContainer" named requirements.
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
                  ciabatta::curry<SimpleEstimation>::template mixin,
                  MotionModel,
                  SensorModel> {
  using ciabatta::mixin<
      AMCL<MotionModel, SensorModel, State, Container>,
      ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
      ciabatta::curry<BaselineGeneration>::template mixin,
      ciabatta::curry<AdaptiveGeneration>::template mixin,
      ciabatta::curry<KldResampling>::template mixin,
      ciabatta::curry<SimpleEstimation>::template mixin,
      MotionModel,
      SensorModel>::mixin;
};

}  // namespace beluga

#endif
