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
#include <random>
#include <utility>

#include <range/v3/view/common.hpp>

/**
 * \file
 * \brief Implementation of particle filters.
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
 * - `p.weights()` is valid and returns a view to a container to the weights of the particles.
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

struct BaseParticleFilterInterface {
  virtual ~BaseParticleFilterInterface() = default;
  virtual void sample() = 0;
  virtual void sample(std::execution::sequenced_policy) { return this->sample(); };
  virtual void sample(std::execution::parallel_policy) { return this->sample(); };
  virtual void importance_sample() = 0;
  virtual void importance_sample(std::execution::sequenced_policy) { return this->importance_sample(); };
  virtual void importance_sample(std::execution::parallel_policy) { return this->importance_sample(); };
  virtual void resample() = 0;
};

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
template <class Mixin>
struct BootstrapParticleFilter : public Mixin {
 public:
  /// Constructs a BootstrapParticleFilter.
  /**
   * The initial particles are generated using the \ref ParticleBaselineGenerationPage "ParticleBaselineGeneration"
   * implementation of `Mixin`.
   *
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit BootstrapParticleFilter(Args&&... args) : Mixin(std::forward<Args>(args)...) {
    this->self().initialize_particles(this->self().generate_samples(generator_) | this->self().take_samples());
  }

  /// Update the particles states based on the motion model and the last pose update.
  /**
   * The update will be done based on the `Mixin` implementation of the
   * \ref MotionModelPage "MotionModel" named requirement.
   */
  void sample() final { return this->sample_impl(std::execution::seq); }
  void sample(std::execution::sequenced_policy policy) final { this->sample_impl(policy); }
  void sample(std::execution::parallel_policy policy) final { this->sample_impl(policy); }

  /// Update the particle weights based on the sensor model.
  /**
   * The update will be done based on the `Mixin` implementation of the \ref SensorModelPage "SensorModel"
   * named requirement.
   */
  void importance_sample() final { this->importance_sample_impl(std::execution::seq); }
  void importance_sample(std::execution::sequenced_policy policy) final { this->importance_sample_impl(policy); }
  void importance_sample(std::execution::parallel_policy policy) final { this->importance_sample_impl(policy); }

  /// Resample particles based on ther weights and the resampling policy used.
  /**
   * The update will be done based on the Mixin implementation of the
   * \ref ParticleSampledGenerationPage "ParticleSampledGeneration" and
   * \ref ParticleResamplingPage "ParticleResampling" named requirements
   * and the active resampling policies.
   */
  void resample() final {
    const auto resampling_vote_result = this->self().do_resampling_vote();
    if (resampling_vote_result) {
      this->self().initialize_particles(
          this->self().generate_samples_from_particles(generator_) | this->self().take_samples());
    }
  }

 private:
  std::mt19937 generator_{std::random_device()()};

  /// Update the particles states based on the motion model and the last pose update.
  template <typename ExecutionPolicy>
  void sample_impl(ExecutionPolicy&& policy) {
    auto states = this->self().states() | ranges::views::common;
    std::transform(
        std::forward<ExecutionPolicy>(policy), std::begin(states), std::end(states), std::begin(states),
        [this](const auto& state) { return this->self().apply_motion(state, generator_); });
  }

  /// Update the particle weights based on the sensor model.
  template <typename ExecutionPolicy>
  void importance_sample_impl(ExecutionPolicy&& policy) {
    auto states = this->self().states() | ranges::views::common;
    std::transform(
        std::forward<ExecutionPolicy>(policy), std::begin(states), std::end(states), std::begin(this->self().weights()),
        [this](const auto& state) { return this->self().importance_weight(state); });
  }
};

}  // namespace beluga

#endif
