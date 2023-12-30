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

#ifndef BELUGA_ALGORITHM_SAMPLING_HPP
#define BELUGA_ALGORITHM_SAMPLING_HPP

#include <functional>
#include <limits>
#include <random>

#include <beluga/algorithm/exponential_filter.hpp>
#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/type_traits.hpp>
#include <beluga/views/particles.hpp>
#include <beluga/views/take_while_kld.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/take_while.hpp>
#include <range/v3/view/transform.hpp>

/**
 * \file
 * \brief Implementation of algorithms related to generation, sampling and resampling
 *  of particles.
 */

namespace beluga {

/**
 * \page StateGeneratorPage Beluga named requirements: StateGenerator
 * Classes satisfying the `StateGenerator` requirements can be used in a particle filter
 * to generate the initial set of particles.
 *
 * \section StateGeneratorRequirements Requirements
 * A type `T` satisfies the `StateGenerator` requirements if `T` is a mixin type, and given:
 * - A type `G` that satisfies the requirements of
 *   [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator).
 * - A type `S` that represents the state of a particle.
 * - An instance `p` of `T`.
 * - An instance `g` of `G`.
 *
 * Then:
 * - `p.generate_samples(g)` returns a [range](https://en.cppreference.com/w/cpp/ranges/range)
 *   whose value type is `S`.
 *
 * \section StateGeneratorLinks See also
 * - beluga::RandomStateGenerator
 */

/**
 * \page SamplerPage Beluga named requirements: Sampler
 * Classes satisfying the `Sampler` requirements can be used in a particle filter
 * to take a new sample of particles from the previous particle set.
 *
 * \section SamplerRequirements Requirements
 * A type `T` satisfies the `Sampler` requirements if `T` is a mixin type, and given:
 * - A type `G` that satisfies the requirements of
 *   [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator).
 * - A type `S` that represents the state of a particle.
 * - An instance `p` of `T`.
 * - An instance `g` of `G`.
 *
 * Then:
 * - `p.generate_samples_from_particles(g)` returns a [range](https://en.cppreference.com/w/cpp/ranges/range)
 *   whose value type is `S`.
 *
 * \section SamplerLinks See also
 * - beluga::NaiveSampler
 * - beluga::AdaptiveSampler
 */

/**
 * \page LimiterPage Beluga named requirements: Limiter
 * Classes satisfying the `Limiter` requirements can be used in the particle filter
 * to provide a policy for deciding how many particles to sample.
 *
 * \section LimiterRequirements Requirements
 * A type `T` satisfies the `Limiter` requirements if given:
 * - A type `S` that represents the state of a particle.
 * - A [range](https://en.cppreference.com/w/cpp/ranges/range) `R` with value type `S`.
 * - A possibly const instance `t` of `T`.
 * - An instance `r` of `R`.
 *
 * Then:
 * - `t.take_samples()` returns a [range adaptor object](
 *   https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject).
 * - If `v` is the return value of `t.take_samples()`,
 *   the expression `r | v` is valid and results in a range view of particles.
 *   This range view contains the particles that result of resampling `r` according
 *   to the policy of `T`.
 * - `t.max_samples()` is a valid expression and returns the maximum number of samples
 *   the policy will generate.
 * - `t.min_samples()` is a valid expression and returns the minimum number of samples
 *   the policy will generate.
 *
 * \section LimiterLinks See also
 * - beluga::FixedLimiter
 * - beluga::KldLimiter
 */

/// Selects between executing one function or another randomly.
/**
 * \tparam Function1 Callable type, with prototype `() -> Ret`.
 * \tparam Function2 Callable type, with prototype `() -> Ret`.
 *   The return type of both Function1 and Function2 must be the same.
 * \tparam RandomNumberGenerator Must meet the requirements of
 *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator).
 * \param first The first function to be called.
 * \param second The second function to be called.
 * \param generator The random number generator used.
 * \param probability The first function will be called with the probability specified here.
 *  In the rest of the cases, second is called.
 *  A Bernoully distribution is used.
 * \return The result of the called function.
 *  The return type is `decltype(first())`.
 */
template <class Function1, class Function2, class RandomNumberGenerator>
auto make_random_selector(Function1&& first, Function2&& second, RandomNumberGenerator& generator, double probability) {
  return [first = std::forward<Function1>(first), second = std::forward<Function2>(second), &generator,
          distribution = std::bernoulli_distribution{probability}]() mutable {
    return distribution(generator) ? first() : second();
  };
}

/// Returns multinomial resampler function.
/**
 * Returns a function that when called picks randomly a sample from the input range, with individual
 * probabilities given by the weights of each.
 * \tparam Range A [Range](https://en.cppreference.com/w/cpp/ranges/range) type, its iterator
 *  must be [random access](https://en.cppreference.com/w/cpp/named_req/RandomAccessIterator).
 * \tparam Weights A [Range](https://en.cppreference.com/w/cpp/ranges/range) type,
 *  its values must be convertible to double.
 * \tparam RandomNumberGenerator Must meet the requirements of
 *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator).
 * \param samples The container of samples to be picked.
 * \param weights The weights of the samples to be picked.
 *  The size of the container must be the same as the size of samples.
 *  For a sample `samples[i]`, its weight is `weights[i]`.
 * \param generator The random number generator used.
 * \return The sampler function.
 *  Its type is the same as the `Range` value type.
 */
template <class Range, class Weights, class RandomNumberGenerator>
auto make_multinomial_sampler(const Range& samples, const Weights& weights, RandomNumberGenerator& generator) {
  auto weights_begin = std::begin(weights);
  auto weights_end = std::end(weights);
  using difference_type = decltype(weights_end - weights_begin);
  return [&generator, first = std::begin(samples),
          distribution = std::discrete_distribution<difference_type>{weights_begin, weights_end}]() mutable {
    return *(first + distribution(generator));
  };
}

/// A random state generator.
/**
 * An implementation of \ref StateGeneratorPage.
 *
 * \tparam Mixin The mixed-in type. An instance `m` of `Mixin` must provide:
 * - A `make_random_state()` method that satisfies the requirements specified
 *   in \ref SensorModelPage.
 */
template <class Mixin>
class RandomStateGenerator : public Mixin {
 public:
  /// Constructs a RandomStateGenerator instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit RandomStateGenerator(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Generates new random state samples.
  /**
   * The states are generated according to the `make_random_state()` method
   * provided by the mixin.
   *
   * \tparam Generator  A random number generator that must satisfy the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param gen An uniform random bit generator object.
   * \return A range view that sources a sequence of random states.
   */
  template <class Generator>
  [[nodiscard]] auto generate_samples(Generator& gen) {
    return ranges::views::generate([this, &gen]() { return this->self().make_random_state(gen); });
  }
};

/// Generation of samples from input particles.
/**
 * An implementation of \ref SamplerPage.
 *
 * Naive implementation that has no way to recover if there are no particles
 * close to the true state.
 *
 * \tparam Mixin The mixed-in type. An instance `m` of `Mixin` must provide:
 * - A `states()` and `weights()` methods that satisfy the requirements
 *   specified in \ref StoragePolicyPage.
 */
template <class Mixin>
class NaiveSampler : public Mixin {
 public:
  /// Constructs a NaiveSampler instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit NaiveSampler(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Returns a view that sources a sequence of with-replacement samples from the existing set.
  /**
   * The new states are generated according to the `states()` and `weights()` methods
   * provided by the mixin.
   *
   * \tparam Generator  A random number generator that must satisfy the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param gen An uniform random bit generator object.
   * \return A range view that consists of with-replacement samples from the existing set.
   */
  template <class Generator>
  [[nodiscard]] auto generate_samples_from_particles(Generator& gen) const {
    return ranges::views::generate(
        beluga::make_multinomial_sampler(this->self().states(), this->self().weights(), gen));
  }
};

/// Parameters used to construct an AdaptiveSampler instance.
struct AdaptiveSamplerParam {
  /// Smoothing coefficient used in the slow exponential filter.
  double alpha_slow;
  /// Smoothing coefficient used in the fast exponential filter.
  double alpha_fast;
};

/// Generation of samples from input particles with a recovery strategy.
/**
 * An implementation of \ref SamplerPage.
 *
 * The addition of random samples allows the filter to recover.
 * It determines how many random particles to add by averaging the weights of the particles.
 * The estimate used considers a short-term and a long-term average.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic, Chapter 8.3.3.
 *
 * \tparam Mixin The mixed-in type. An instance `m` of `Mixin` must provide:
 * - A `states()` and `weights()`  methods that satisfy
 *   the requirements specified in \ref StoragePolicyPage.
 * - A `make_random_state()` method that satisfies the requirements specified in
 *   \ref SensorModelPage.
 */
template <class Mixin>
class AdaptiveSampler : public Mixin {
 public:
  /// Parameter type that the constructor uses to configure the generation.
  using param_type = AdaptiveSamplerParam;

  /// Constructs an AdaptiveSampler instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param parameters Parameters to configure the instance.
   *  See beluga::AdaptiveSamplerParam for details.
   * \param ...rest Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit AdaptiveSampler(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), slow_filter_{parameters.alpha_slow}, fast_filter_{parameters.alpha_fast} {}

  /// Generates new samples from the current particles.
  /**
   * A random state will be generated with a probability of
   * `P = max(0, 1 - fast_filter_average / slow_filter_average)`
   * where the filters are configured according to `param_type` specified in the constructor.
   * If not, an existing particle will be sampled.
   *
   * The filters are reset if P > 0 for the next iteration, to avoid spiraling off into complete randomness.
   *
   * \tparam Generator  A random number generator that must satisfy the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param gen An uniform random bit generator object.
   * \return A range view that consists of a mixture of with-replacement samples from the existing set and brand new
   * random samples.
   */
  template <class Generator>
  [[nodiscard]] auto generate_samples_from_particles(Generator& gen) {
    auto weights = this->self().weights() | ranges::views::common;
    double total_weight = std::reduce(std::begin(weights), std::end(weights), 0.);
    double average_weight = total_weight / static_cast<double>(weights.size());
    double random_state_probability = std::max(0., 1. - fast_filter_(average_weight) / slow_filter_(average_weight));

    if (random_state_probability > 0.) {
      fast_filter_.reset();
      slow_filter_.reset();
    }

    auto create_random_state = [this, &gen] { return this->self().make_random_state(gen); };
    auto sample_existing_state = make_multinomial_sampler(this->self().states(), weights, gen);
    return ranges::views::generate(make_random_selector(
        std::move(create_random_state), std::move(sample_existing_state), gen, random_state_probability));
  }

 private:
  ExponentialFilter slow_filter_;
  ExponentialFilter fast_filter_;
};

/// Parameters used to construct a FixedLimiter instance.
struct FixedLimiterParam {
  /// Maximum number of particles to be sampled.
  std::size_t max_samples;
};

/// Limiter policy that takes a fixed number of particles.
/**
 * An implementation of \ref LimiterPage.
 *
 * \tparam Mixin The mixed-in type. `Mixin::self_type::particle_type` must exist and
 * satisfy \ref ParticlePage.
 */
template <class Mixin>
class FixedLimiter : public Mixin {
 public:
  /// Parameters type used to construct an instance of this class.
  using param_type = FixedLimiterParam;

  /// Constructs a FixedLimiter instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param parameters Parameters to configure this instance.
   *  See beluga::FixedLimiterParam for details.
   * \param ...rest Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit FixedLimiter(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), parameters_{parameters} {}

  /// Returns the minimum number of particles to be sampled.
  /**
   * This policy uses a fixed number of particles, so the return value is equal
   * to `max_samples()` and also to `FixedLimiterParam::max_samples` parameter
   * specified in the constructor.
   *
   * \return Minimum number of particles to be sampled.
   */
  [[nodiscard]] std::size_t min_samples() const { return parameters_.max_samples; }

  /// Returns the maximum number of particles to be sampled.
  /**
   * This policy uses a fixed number of particles, so the return value is equal
   * to `min_samples()` and also to `FixedLimiterParam::max_samples` parameter
   * specified in the constructor.
   *
   * \return Maximum number of particles to be sampled.
   */
  [[nodiscard]] std::size_t max_samples() const { return parameters_.max_samples; }

  /// Takes samples from a range of states.
  /**
   * The returned range adaptor object can be composed with any range of particle states.
   *
   * \return A [RangeAdaptorObject](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
   * that will sample until the required number of particles is reached.
   */
  [[nodiscard]] auto take_samples() const { return ranges::views::take_exactly(parameters_.max_samples); }

 private:
  param_type parameters_;
};

/// Parameters used to construct a KldLimiter instance.
/**
 * \tparam State Type that represents the state of the particle.
 */
template <class State>
struct KldLimiterParam {
  /// Minimum number of particles to be sampled.
  std::size_t min_samples;
  /// Maximum number of particles to be sampled.
  std::size_t max_samples;
  /// Hasher instance used to compute the spatial cluster for a given state.
  spatial_hash<State> spatial_hasher;
  /// See beluga::kld_condition() for details.
  double kld_epsilon;
  /// See beluga::kld_condition() for details.
  double kld_z;
};

/// Limiter policy that takes a number of particles defined by the KLD criteria.
/**
 *
 * An implementation of \ref LimiterPage.
 *
 * See KLD-Sampling: Adaptive Particle Filters \cite fox2001adaptivekldsampling.
 *
 * \tparam Mixin The mixed-in type. `Mixin::self_type::particle_type` must exist and
 * satisfy \ref ParticlePage.
 * \tparam State Type that represents the state of a particle. It should match with
 * particle_traits<Mixin::self_type::particle_type>::state_type.
 *
 * Additionally, given:
 * - `P`, the type `Mixin::self_type::particle_type`
 * - `p` an instance of `P`
 * - `cp` a possibly const instance of `P`
 * - `h` an instance of `std::size_t`
 */
template <class Mixin, class State>
class KldLimiter : public Mixin {
 public:
  /// Parameters type used to construct a KldLimiter instance.
  using param_type = KldLimiterParam<State>;

  /// Constructs a KldLimiter instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param parameters Parameters to configure this instance.
   *  See beluga::KldLimiterParam for details.
   * \param ...rest Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit KldLimiter(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), parameters_{parameters} {}

  /// Returns the minimum number of particles to be sampled.
  /**
   * This is equal to what was specified in the KldLimiterParam::min_samples
   * parameter passed in the constructor.
   *
   * \return Minimum number of particles to be sampled.
   */
  [[nodiscard]] std::size_t min_samples() const { return parameters_.min_samples; }

  /// Returns the maximum number of particles to be sampled.
  /**
   * This is equal to what was specified in the KldLimiterParam::max_samples
   * parameter passed in the constructor.
   *
   * \return Maximum number of particles to be sampled.
   */
  [[nodiscard]] std::size_t max_samples() const { return parameters_.max_samples; }

  /// Takes samples from a range of states.
  /**
   * The returned range adaptor object can be composed with any range of particle states.
   *
   * \return A [RangeAdaptorObject](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
   * that will sample until the KLD criteria is satisfied.
   */
  [[nodiscard]] auto take_samples() const {
    return beluga::views::take_while_kld(
        parameters_.spatial_hasher,  //
        parameters_.min_samples,     //
        parameters_.max_samples,     //
        parameters_.kld_epsilon,     //
        parameters_.kld_z);
  }

 private:
  param_type parameters_;
};

}  // namespace beluga

#endif
