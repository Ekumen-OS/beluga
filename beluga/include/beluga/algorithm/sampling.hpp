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
 * \page ParticleBaselineGenerationPage beluga named requirements: ParticleBaselineGeneration
 * Classes satisfying the `ParticleBaselineGeneration` requirements can be used in a particle filter
 * to generate the initial set of particles.
 *
 * \section ParticleBaselineGenerationRequirements Requirements
 * A type `T` satisfies the `ParticleBaselineGeneration` requirements if given:
 * - A type `P` that satisfies the \ref ParticlePage "Particle" named requirements.
 * - An instance `g` of `T`.
 *
 * Then:
 * - `g.generate_samples<P>()` returns a [range](https://en.cppreference.com/w/cpp/ranges/range) of particles `P`.
 */

/**
 * \page ParticleSampledGenerationPage beluga named requirements: ParticleSampledGeneration
 * Classes satisfying the `ParticleSampledGeneration` requirements can be used in a particle filter
 * to generate new particles from the previous particles set.
 *
 * \section ParticleSampledGenerationRequirements Requirements
 * A type `T` satisfies the `ParticleSampledGeneration` requirements if given:
 * - A type `P` that satisfies the \ref ParticlePage "Particle" named requirements.
 * - An instance `p` of a [range](https://en.cppreference.com/w/cpp/ranges/range) of particles of
 *   type `P`.
 * - An instance `g` of `T`.
 *
 * Then:
 * - `g.generate_samples_from(p)` returns a [range](https://en.cppreference.com/w/cpp/ranges/range) of particles `P`.
 *   The input range `p` may be used as a base to generate the particles in the returned range.
 */

/**
 * \page ParticleResamplingPage beluga named requirements: ParticleResampling
 * Classes satisfying the `ParticleResampling` can be used in the particle filter to provide a policy
 * of how the previous particles are resampled.
 *
 * \section ParticleResamplingRequirements Requirements
 * A type `T` satisfies the `ParticleResampling` requirements if given:
 * - A type `P` that satisfies the \ref ParticlePage "Particle" named requirements.
 *   Particular implementations may have extra requirements on `P`.
 * - A [range](https://en.cppreference.com/w/cpp/ranges/range) `R` with value type `P`.
 * - A possibly const instance `t` of `T`.
 *
 * Then:
 * - `t.take_samples()` returns a [range adaptor object](
 *   https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject).
 * - If `r` is an instance of `R` and `v` the return values of `t.take_samples()`,
 *   the expression `r | v` is valid and results in a range view of particles.
 *   This range view contains the particles that result of resampling `r` according
 *   to the policy of `T`.
 */

/// Selects between executing one function or another randomly.
/**
 * \tparam Function1 Callable type, with prototype () -> Ret.
 * \tparam Function2 Callable type, with prototype () -> Ret.
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
 *  The return type is decltype(first()).
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
 *  For a sample samples[i], its weight is weights[i].
 * \param generator The random number generator used.
 * \return The picked sample.
 *  Its type is the same as the Range value type.
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
 * \return A callable object with prototype (ParticleT && p) -> ParticleT.
 *  ParticleT must satisfy the \ref ParticlePage "Particle" named requirements.
 *  The expression particle_traits<ParticleT>::cluster(p) must also
 *  be valid and return a `std::size_t &`.
 *  After the returned object is applied to a particle p, cluster(p) will be updated
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

/// Returns a callable object that verifies if the kld condition is being satisfied.
/**
 * The callable object will compute the minimum number of samples based on a Kullback-Leibler
 * distance epsilon between the maximum likelihood estimate and the true distribution.
 * Z is the upper standard normal quantile for P, where P is the probability
 * that the error in the estimated distribution will be less than epsilon.
 * Here are some examples:
 * - P = 0.900 -> Z = 1.28155156327703
 * - P = 0.950 -> Z = 1.64485362793663
 * - P = 0.990 -> Z = 2.32634787735669
 * - P = 0.999 -> Z = 3.09023224677087
 *
 * If the computed value is less than what the min argument specifies, then min will be returned.
 *
 * \param min Minimum number of particles that the callable object will return.
 * \param epsilon Maximum distance epsilon between the maximum likelihood estimate and the true
 *  distrubution.
 * \param z Upper standard normal quantile for the probability that the error in the
 *  estimated distribution is less than epsilon.
 * \return A callable object with prototype (std::size_t hash) -> bool.
 *  hash is the spatial hash of the particle being added. \n
 *  The returned callable object is stateful, tracking the total number of particles and
 *  the particle clusters based on the spatial hash. \n
 *  The return value of the callable will be false when the number of particles is more than the minimum
 *  and the kld condition is satisfied, if not it will be true. \n
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

/// A particle generator.
/**
 * An implementation of the \ref ParticleBaselineGenerationPage "ParticleBaselineGeneration"
 * named requirements.
 *
 * \tparam Mixin The mixed-in type. An instance m of Mixin must provide a protected method,
 *  m.self(). The return type of m.self() must satisfy the SensorModel named requirements.
 * \tparam RandomNumberGenerator A random number generator, must satisfy the
 *  [UniformRandomBitGenerator](
 *  https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator) requirements.
 */
template <class Mixin, class RandomNumberGenerator = typename std::mt19937>
struct BaselineGeneration : public Mixin {
 public:
  /// Constructs a BaselineGeneration instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit BaselineGeneration(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Returns a range containing the generated particles.
  /**
   * The particles are generated randomly according to the `generate_random_state()` method provided by the sensor
   * model. See \ref SensorModelPage "SensorModel".
   *
   * \tparam Particle The particle type, must satisfy the \ref ParticlePage "Particle" named requirements.
   * \return range with the generated particles.
   */
  template <class Particle>
  [[nodiscard]] auto generate_samples() {
    return ranges::views::generate([this]() { return this->self().generate_random_state(random_number_generator_); }) |
           ranges::views::transform(make_from_state<Particle>);
  }

 private:
  RandomNumberGenerator random_number_generator_{std::random_device()()};
};

/// Generation of samples from input particles.
/**
 * \tparam Mixin The mixed-in type.
 * \tparam RandomNumberGenerator A random number generator, must satisfy the
 *  [UniformRandomBitGenerator](
 *  https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator) requirements.
 */
template <class Mixin, class RandomNumberGenerator = typename std::mt19937>
struct NaiveGeneration : public Mixin {
 public:
  /// Constructs a NaiveGeneration instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit NaiveGeneration(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Generates new particles from the given input particles.
  /**
   * \tparam Range A range of particles. The value type of the range must satisfy the
   *  \ref ParticlePage "Particle" named requirements.
   * \param particles The input particles from where the output particles are sampled.
   * \return The range of sampled particles.
   */
  template <class Range>
  [[nodiscard]] auto generate_samples_from(Range&& particles) {
    return ranges::views::generate(
        random_sample(views::all(particles), views::weights(particles), random_number_generator_));
  }

 private:
  RandomNumberGenerator random_number_generator_{std::random_device()()};
};

/// Parameters used to construct an AdaptiveGeneration instance.
struct AdaptiveGenerationParam {
  /// Smoothing coefficient used in the slow exponential filter.
  double alpha_slow;
  /// Smoothing coefficient used in the fast exponential filter.
  double alpha_fast;
};

/// Generation of samples from input particles, adding random particles if all remaining particles
/// are not close to the correct state.
/**
 * The addition of random samples allows the filter to recover.
 * It determines how many random particles to add by averaging the weights of the particles.
 * The estimate used considers a short-term and a long-term average.
 * See Probabilistic Robotics \cite thrun2005probabilistic, Chapter `8.3.3`.
 *
 * \tparam Mixin The mixed-in type. An instance m of Mixin must provide a protected method,
 *  m.self(). The return type of m.self() must satisfy the SensorModel named requirements.
 * \tparam RandomNumberGenerator A random number generator, must satisfy the
 *  [UniformRandomBitGenerator](
 *  https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator) requirements.
 */
template <class Mixin, class RandomNumberGenerator = typename std::mt19937>
struct AdaptiveGeneration : public Mixin {
 public:
  /// Parameter type that the constructor uses to configure the generation.
  using param_type = AdaptiveGenerationParam;

  /// Constructs an AdaptiveGeneration instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param parameters Parameters to configure the instance.
   *  See beluga::AdaptiveGenerationParam for details.
   * \param ...rest arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit AdaptiveGeneration(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), slow_filter_{parameters.alpha_slow}, fast_filter_{parameters.alpha_fast} {}

  /// Generates new particles from the given input particles.
  /**
   * A random state will be generated with a probability of
   * P = max(0, 1 - fast_filter_average / slow_filter_average)
   * where the filters are configured according to param_type specified in the constructor.
   * If not, a previous particle will be sampled.
   *
   * The filters are reset if P > 0 for the next iteration, to avoid spiraling off into complete randomness.
   *
   * \tparam Range A range of particles. The value type of the range must satisfy the
   *  \ref ParticlePage "Particle" named requirements.
   * \param particles The input particles from where the output particles are sampled.
   * \return The range of sampled particles.
   */
  template <class Range>
  [[nodiscard]] auto generate_samples_from(Range&& particles) {
    auto&& weights = views::weights(particles);
    double total_weight = std::reduce(std::begin(weights), std::end(weights), 0.);
    double average_weight = total_weight / static_cast<double>(weights.size());
    double random_state_probability = std::max(0., 1. - fast_filter_(average_weight) / slow_filter_(average_weight));

    if (random_state_probability > 0.) {
      fast_filter_.reset();
      slow_filter_.reset();
    }

    using particle_type = typename std::decay_t<Range>::value_type;
    return ranges::views::generate(random_select(
               [this]() { return this->self().generate_random_state(random_number_generator_); },
               random_sample(views::states(particles), weights, random_number_generator_), random_number_generator_,
               random_state_probability)) |
           ranges::views::transform(make_from_state<particle_type>);
  }

 private:
  RandomNumberGenerator random_number_generator_{std::random_device()()};
  ExponentialFilter slow_filter_;
  ExponentialFilter fast_filter_;
};

/// Parameters used to construct a FixedResamplingParam instance.
struct FixedResamplingParam {
  /// Maximum number of particles to be sampled.
  std::size_t max_samples;
};

/// Resampling policy that uses a fixed number of particles.
/**
 * \tparam Mixin The mixed-in type.
 */
template <class Mixin>
struct FixedResampling : public Mixin {
 public:
  /// Parameters type used to construct an instance of this class.
  using param_type = FixedResamplingParam;

  /// Constructs a FixedResampling instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param parameters Parameters to configure this instance.
   *  See beluga::FixedResamplingParam for details.
   * \param ...rest arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit FixedResampling(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), parameters_{parameters} {}

  /// Returns the minimum number of particles to be sampled.
  /**
   * This policy uses a fixed number of particles, so the return value is equal
   * to this->max_samples() and also to FixedResamplingParam::max_samples parameter
   * specified in the constructor.
   *
   * \return Minimum number of particles to be sampled.
   */
  [[nodiscard]] std::size_t min_samples() const { return parameters_.max_samples; }
  /// Returns the maximum number of particles to be sampled.
  /**
   * This policy uses a fixed number of particles, so the return value is equal
   * to this->min_samples() and also to FixedResamplingParam::max_samples parameter
   * specified in the constructor.
   *
   * \return Maximum number of particles to be sampled.
   */
  [[nodiscard]] std::size_t max_samples() const { return parameters_.max_samples; }

  /// Returns a [RangeAdaptorObject](
  /// https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject), that will sample until the
  /// required number of particles is reached.
  /**
   * The returned range adaptor object can be composed with any particle range.
   */
  [[nodiscard]] auto take_samples() const { return ranges::views::take_exactly(parameters_.max_samples); }

 private:
  param_type parameters_;
};

/// Parameters used to construct a KldResampling instance.
struct KldResamplingParam {
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

/// Resampling policy that adapts the number of particles according to the KLD criteria.
/**
 * \tparam Mixin The mixed-in type.
 */
template <class Mixin>
struct KldResampling : public Mixin {
 public:
  /// Parameters type used to construct a KldResampling instance.
  using param_type = KldResamplingParam;

  /// Constructs a KldResampling instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param parameters Parameters to configure this instance.
   *  See beluga::KldResamplingParam for details.
   * \param ...rest arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit KldResampling(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), parameters_{parameters} {}

  /// Returns the minimum number of particles to be sampled.
  /**
   * This is equal to what was specified in the KldResamplingParam::min_samples
   * parameter passed in the constructor.
   *
   * \return Minimum number of particles to be sampled.
   */
  [[nodiscard]] std::size_t min_samples() const { return parameters_.min_samples; }
  /// Returns the maximum number of particles to be sampled.
  /**
   * This is equal to what was specified in the KldResamplingParam::max_samples
   * parameter passed in the constructor.
   *
   * \return Maximum number of particles to be sampled.
   */
  [[nodiscard]] std::size_t max_samples() const { return parameters_.max_samples; }

  /// Returns a [RangeAdaptorObject](
  /// https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject), that will sample until the
  /// kld condition is satisfied.
  /**
   * The returned range adaptor object can be composed with a particle range.
   * It can only be composed with a range whose value type satisfies:
   * - The \ref ParticlePage "Particle" named requirements.
   * - Given `P` the range value type, `p` an instance of `P`, `cp` a possibly const instance of `P`.
   *   - The expression \c particle_traits<P>::cluster(cp) returns a `std::size_t` that represents the spatial
   *     hash of the particle `cp`.
   *   - Given a `std::size_t` hash, the expression \c particle_traits<P>::cluster(p) = hash is valid and
   *     assigns the cluster hash to the particle `p`. \n
   *     i.e. after the assignment hash == \c particle_traits<P>::cluster(p) is `true`.
   */
  [[nodiscard]] auto take_samples() const {
    return ranges::views::transform(set_cluster(parameters_.spatial_resolution)) |
           ranges::views::take_while(
               kld_condition(parameters_.min_samples, parameters_.kld_epsilon, parameters_.kld_z),
               [](auto&& particle) { return cluster(particle); }) |
           ranges::views::take(parameters_.max_samples);
  }

 private:
  param_type parameters_;
};

}  // namespace beluga

#endif
