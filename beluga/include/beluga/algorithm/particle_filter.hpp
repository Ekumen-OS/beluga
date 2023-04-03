// Copyright 2022-2023 Ekumen, Inc.
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
 * \page BaseParticleFilterPage Beluga named requirements: BaseParticleFilter
 * Base requirements of a particle filter in Beluga.
 *
 * \section BaseParticleFilterRequirements Requirements
 * `B` is a `BaseParticleFilter` if given:
 * - `b`, a value of type `B`.
 *
 * The following is satisfied:
 * - `b.sample()` updates the particles based on the latest motion update.
 * - `b.reweight()` updates the particles weight based on the latest sensor update.
 * - `b.resample()` generates new particles from the old ones based on their importance weights.
 *
 * \section BaseParticleFilterLinks See also
 * - beluga::BootstrapParticleFilter
 */

namespace beluga {

/// Pure abstract class representing the base particle filter interface.
struct BaseParticleFilterInterface {
  /// Virtual destructor.
  virtual ~BaseParticleFilterInterface() = default;

  /// Update the states of the particles.
  /**
   * This step generates a hyphotetical state based on the current
   * particle state and controls, while leaving their importance weights unchanged.
   */
  virtual void sample() = 0;

  /**
   * \overload
   * It allows specifying a sequenced execution policy.
   */
  virtual void sample(std::execution::sequenced_policy) { return this->sample(); };

  /**
   * \overload
   * It allows specifying a parallel execution policy.
   */
  virtual void sample(std::execution::parallel_policy) { return this->sample(); };

  /// Update the weights of the particles.
  /**
   * This step computes the importance factor or weight of each particle
   * to incorporate measurements. The importance is proportional to the
   * probability of seeing the measurement given the current particle state.
   */
  virtual void reweight() = 0;

  /**
   * \overload
   * It allows specifying a sequenced execution policy.
   */
  virtual void reweight(std::execution::sequenced_policy) { return this->reweight(); };

  /**
   * \overload
   * It allows specifying a parallel execution policy.
   */
  virtual void reweight(std::execution::parallel_policy) { return this->reweight(); };

  /// Resample particles based on their weights.
  /**
   * This step creates a new particle set drawing samples from the
   * current particle set. The probability of drawing each particle
   * is given by its importance weight.
   *
   * The resampling process can be inhibited by the resampling policies.
   *
   * If resampling is performed, the importance weights of all particles are reset to 1.0.
   */
  virtual void resample() = 0;

  /// Distribute the particles over all the space.
  /**
   * \overload
   * this step will reinitialize the position of each particle based
   * in some predefined distribution.
   */
  virtual void reinitialize() = 0;
};

/// Base implementation of a particle filter.
/**
 * This class implements BaseParticleFilterInterface and satisfies the \ref BaseParticleFilterPage.
 *
 * \tparam Mixin The mixed-in type. An instance `m` of `Mixin` must provide:
 * - A `states()`, `weights()` and `initialize_particles()` methods that
 *   satisfy the requirements specified in \ref StoragePolicyPage.
 * - A `generate_samples()` method that satisfies the requirements specified in
 *  \ref StateGeneratorPage.
 * - A `generate_samples_from_particles()` method that satisfies the requirements
 *  specified in \ref SamplerPage.
 * - A `take_samples()` method that satisfies the requirements specified in
 *  \ref LimiterPage.
 * - An `apply_motion()` method that satisfies the requirements specified in
 *  \ref MotionModelPage.
 * - An `importance_weight()` method that satisfies the requirements specified in
 *  \ref SensorModelPage.
 */
template <class Mixin>
class BootstrapParticleFilter : public Mixin {
 public:
  /// Constructs a BootstrapParticleFilter.
  /**
   * The initial particles are generated using the \ref StateGeneratorPage "StateGenerator"
   * implementation of `Mixin`.
   *
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit BootstrapParticleFilter(Args&&... args) : Mixin(std::forward<Args>(args)...) {
    reinitialize();
  }
  /*
   Distribute the particles using the \ref StateGeneratorPage "StateGenerator"
  */
  void reinitialize() final {
    this->self().initialize_particles(this->self().generate_samples(generator_) | this->self().take_samples());
  }

  /**
   * \copydoc BaseParticleFilterInterface::sample()
   *
   * The update will be done based on the `Mixin` implementation of the
   * \ref MotionModelPage "MotionModel" named requirements.
   */
  void sample() final { return this->sample_impl(std::execution::seq); }

  /// \copydoc BaseParticleFilterInterface::sample(std::execution::sequenced_policy policy)
  void sample(std::execution::sequenced_policy policy) final { this->sample_impl(policy); }

  /// \copydoc BaseParticleFilterInterface::sample(std::execution::parallel_policy policy)
  void sample(std::execution::parallel_policy policy) final { this->sample_impl(policy); }

  /**
   * \copydoc BaseParticleFilterInterface::reweight()
   *
   * The update will be done based on the `Mixin` implementation of the
   * \ref SensorModelPage "SensorModel" named requirements.
   */
  void reweight() final { this->reweight_impl(std::execution::seq); }

  /// \copydoc BaseParticleFilterInterface::reweight(std::execution::sequenced_policy policy)
  void reweight(std::execution::sequenced_policy policy) final { this->reweight_impl(policy); }

  /// \copydoc BaseParticleFilterInterface::reweight(std::execution::parallel_policy policy)
  void reweight(std::execution::parallel_policy policy) final { this->reweight_impl(policy); }

  /**
   * \copydoc BaseParticleFilterInterface::resample()
   *
   * The update will be done based on the `Mixin` implementation of the
   * \ref SamplerPage "Sampler" and \ref LimiterPage "Limiter" named requirements
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

  template <typename ExecutionPolicy>
  void sample_impl(ExecutionPolicy&& policy) {
    auto states = this->self().states() | ranges::views::common;
    std::transform(
        std::forward<ExecutionPolicy>(policy), std::begin(states), std::end(states), std::begin(states),
        [this](const auto& state) { return this->self().apply_motion(state, generator_); });
  }

  template <typename ExecutionPolicy>
  void reweight_impl(ExecutionPolicy&& policy) {
    auto states = this->self().states() | ranges::views::common;
    auto weights = this->self().weights() | ranges::views::common;
    std::transform(
        std::forward<ExecutionPolicy>(policy), std::begin(states), std::end(states), std::begin(weights),
        std::begin(weights),
        [this](const auto& state, auto weight) { return weight * this->self().importance_weight(state); });
  }
};

}  // namespace beluga

#endif
