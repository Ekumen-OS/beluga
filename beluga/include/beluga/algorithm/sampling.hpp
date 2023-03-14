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

#ifndef BELUGA_ALGORITHM_SAMPLING_HPP
#define BELUGA_ALGORITHM_SAMPLING_HPP

#include <functional>
#include <limits>
#include <random>
#include <unordered_set>

#include <beluga/algorithm/exponential_filter.hpp>
#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/type_traits.hpp>
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
auto random_select(Function1 first, Function2 second, RandomNumberGenerator& generator, double probability) {
  return [first = std::move(first), second = std::move(second), &generator,
          distribution = std::bernoulli_distribution{probability}]() mutable {
    return distribution(generator) ? first() : second();
  };
}

/// Picks a sample randomly from a range according to the weights of the samples.
/**
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
 * \return The picked sample.
 *  Its type is the same as the `Range` value type.
 */
template <class Range, class Weights, class RandomNumberGenerator>
auto random_sample(const Range& samples, const Weights& weights, RandomNumberGenerator& generator) {
  return [&generator, first = std::begin(samples),
          distribution = std::discrete_distribution<std::size_t>{std::begin(weights), std::end(weights)}]() mutable {
    return *(first + distribution(generator));
  };
}

/// Returns a callable object that allows setting the cluster of a particle.
/**
 * \param resolution The size along any axis of the spatial cluster cell.
 * \return A callable object with prototype `(ParticleT && p) -> ParticleT`. \n
 *  `ParticleT` must satisfy \ref ParticlePage. \n
 *  The expression \c particle_traits<ParticleT>::cluster(p) must also
 *  be valid and return a `std::size_t &`. \n
 *  After the returned object is applied to a particle `p`, \c cluster(p) will be updated
 *  with the calculated spatial hash according to the specified resolution.
 */
inline auto set_cluster(double resolution) {
  return [resolution](auto&& particle) {
    using state_type = std::decay_t<decltype(state(particle))>;
    auto new_particle = particle;
    cluster(new_particle) = spatial_hash<state_type>{}(state(particle), resolution);
    return new_particle;
  };
}

/// Returns a callable object that verifies if the KLD condition is being satisfied.
/**
 * The callable object will compute the minimum number of samples based on a Kullback-Leibler
 * distance epsilon between the maximum likelihood estimate and the true distribution. \n
 * Z is the upper standard normal quantile for P, where P is the probability
 * that the error in the estimated distribution will be less than epsilon.
 *
 * Here are some examples:
 * | P     | Z                |
 * |-------|------------------|
 * | 0.900 | 1.28155156327703 |
 * | 0.950 | 1.64485362793663 |
 * | 0.990 | 2.32634787735669 |
 * | 0.999 | 3.09023224677087 |
 *
 * If the computed value is less than what the min argument specifies, then min will be returned.
 *
 * See KLD-Sampling: Adaptive Particle Filters \cite fox2001adaptivekldsampling.
 *
 * \param min Minimum number of particles that the callable object will return.
 * \param epsilon Maximum distance epsilon between the maximum likelihood estimate and the true
 *  distrubution.
 * \param z Upper standard normal quantile for the probability that the error in the
 *  estimated distribution is less than epsilon.
 * \return A callable object with prototype `(std::size_t hash) -> bool`.
 *  `hash` is the spatial hash of the particle being added. \n
 *  The returned callable object is stateful, tracking the total number of particles and
 *  the particle clusters based on the spatial hash. \n
 *  The return value of the callable will be false when the number of particles is more than the minimum
 *  and the KLD condition is satisfied, if not it will be true. \n
 *  i.e. A return value of true means that you need to keep sampling to satisfy the condition.
 */
inline auto kld_condition(std::size_t min, double epsilon, double z = 3.) {
  auto target_size = [two_epsilon = 2 * epsilon, z](std::size_t k) {
    if (k <= 2U) {
      return std::numeric_limits<std::size_t>::max();
    }
    double common = 2. / static_cast<double>(9 * (k - 1));
    double base = 1. - common + std::sqrt(common) * z;
    double result = (static_cast<double>(k - 1) / two_epsilon) * base * base * base;
    return static_cast<std::size_t>(std::ceil(result));
  };

  return [=, count = 0ULL, buckets = std::unordered_set<std::size_t>{}](std::size_t hash) mutable {
    count++;
    buckets.insert(hash);
    return count <= min || count <= target_size(buckets.size());
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
struct RandomStateGenerator : public Mixin {
 public:
  /// Constructs a RandomStateGenerator instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit RandomStateGenerator(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Generates new random states.
  /**
   * The states are generated according to the `make_random_state()` method
   * provided by the mixin.
   *
   * \tparam Generator  A random number generator that must satisfy the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param gen An uniform random bit generator object.
   * \return A range view that can generate random states.
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
struct NaiveSampler : public Mixin {
 public:
  /// Constructs a NaiveSampler instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit NaiveSampler(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Generates new samples from the current particles.
  /**
   * The new states are generated according to the `states()` and `weights()` methods
   * provided by the mixin.
   *
   * \tparam Generator  A random number generator that must satisfy the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param gen An uniform random bit generator object.
   * \return A range view that can generate samples from the current set of particles.
   */
  template <class Generator>
  [[nodiscard]] auto generate_samples_from_particles(Generator& gen) const {
    return ranges::views::generate(beluga::random_sample(this->self().states(), this->self().weights(), gen));
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
struct AdaptiveSampler : public Mixin {
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
   * \return A range view that can generate samples from the current set of particles.
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

    return ranges::views::generate(random_select(
        [this, &gen]() { return this->self().make_random_state(gen); },
        random_sample(this->self().states(), weights, gen), gen, random_state_probability));
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
struct FixedLimiter : public Mixin {
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
  [[nodiscard]] auto take_samples() const {
    using particle_type = typename Mixin::self_type::particle_type;
    return ranges::views::transform(beluga::make_from_state<particle_type>) |
           ranges::views::take_exactly(parameters_.max_samples);
  }

 private:
  param_type parameters_;
};

/// Parameters used to construct a KldLimiter instance.
struct KldLimiterParam {
  /// Minimum number of particles to be sampled.
  std::size_t min_samples;
  /// Maximum number of particles to be sampled.
  std::size_t max_samples;
  /// Cluster resolution in any axis, used to compute the spatial hash.
  double spatial_resolution;
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
 *
 * Additionally, given:
 * - `P`, the type `Mixin::self_type::particle_type`
 * - `p` an instance of `P`
 * - `cp` a possibly const instance of `P`
 * - `h` an instance of `std::size_t`
 *
 * The following conditions must be satisfied:
 * - The expression \c particle_traits<P>::cluster(cp) returns a `std::size_t` that represents the spatial
 *   hash of the particle `cp`.
 * - The expression \c particle_traits<P>::cluster(p) = `h` is valid and
 *   assigns the cluster hash to the particle `p`. \n
 *   i.e. after the assignment `h` == \c particle_traits<P>::cluster(p) is true.
 */
template <class Mixin>
struct KldLimiter : public Mixin {
 public:
  /// Parameters type used to construct a KldLimiter instance.
  using param_type = KldLimiterParam;

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
    using particle_type = typename Mixin::self_type::particle_type;
    return ranges::views::transform(beluga::make_from_state<particle_type>) |
           ranges::views::transform(beluga::set_cluster(parameters_.spatial_resolution)) |
           ranges::views::take_while(
               beluga::kld_condition(parameters_.min_samples, parameters_.kld_epsilon, parameters_.kld_z),
               [](auto&& particle) { return cluster(particle); }) |
           ranges::views::take(parameters_.max_samples);
  }

 private:
  param_type parameters_;
};

}  // namespace beluga

#endif
