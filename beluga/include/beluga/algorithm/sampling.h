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

#pragma once

#include <functional>
#include <limits>
#include <random>
#include <unordered_set>

#include <range/v3/view/generate.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/take_while.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga/algorithm/average_filter.h>
#include <beluga/spatial_hash.h>
#include <beluga/type_traits.h>

namespace beluga {

template <class Function1, class Function2, class RandomNumberGenerator>
auto random_select(Function1 first, Function2 second, RandomNumberGenerator& generator, double probability) {
  return [first, second, &generator, distribution = std::bernoulli_distribution{probability}]() mutable {
    return distribution(generator) ? first() : second();
  };
}

template <class Range, class Weights, class RandomNumberGenerator>
auto random_sample(const Range& samples, const Weights& weights, RandomNumberGenerator& generator) {
  return [&generator, first = std::begin(samples),
          distribution = std::discrete_distribution<std::size_t>{std::begin(weights), std::end(weights)}]() mutable {
    return *(first + distribution(generator));
  };
}

inline auto set_cluster(double resolution) {
  return [resolution](auto&& particle) {
    using state_type = std::decay_t<decltype(state(particle))>;
    auto new_particle = particle;
    cluster(new_particle) = spatial_hash<state_type>{}(state(particle), resolution);
    return new_particle;
  };
}

inline auto kld_condition(std::size_t min, double epsilon, double z = 3.) {
  // Compute minimum number of samples based on a Kullback-Leibler distance epsilon
  // between the maximum likelihood estimate and the true distribution.
  // Z is the upper standard normal quantile for P, where P is the probability
  // that the error in the estimated distribution will be less than epsilon.
  // Here are some examples:
  //     P = 0.900 -> Z = 1.28155156327703
  //     P = 0.950 -> Z = 1.64485362793663
  //     P = 0.990 -> Z = 2.32634787735669
  //     P = 0.999 -> Z = 3.09023224677087
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

template <class Mixin, class RandomNumberGenerator = typename std::mt19937>
struct BaselineGeneration : public Mixin {
 public:
  template <class... Args>
  explicit BaselineGeneration(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  template <class Particle>
  [[nodiscard]] auto generate_samples() {
    return ranges::views::generate([this]() { return this->self().generate_random_state(random_number_generator_); }) |
           ranges::views::transform(make_from_state<Particle>);
  }

 private:
  RandomNumberGenerator random_number_generator_{std::random_device()()};
};

template <class Mixin, class RandomNumberGenerator = typename std::mt19937>
struct NaiveGeneration : public Mixin {
 public:
  template <class... Args>
  explicit NaiveGeneration(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  template <class Range>
  [[nodiscard]] auto generate_samples_from(Range&& particles) {
    return ranges::views::generate(
        random_sample(views::all(particles), views::weights(particles), random_number_generator_));
  }

 private:
  RandomNumberGenerator random_number_generator_{std::random_device()()};
};

struct AdaptiveGenerationParam {
  double alpha_slow;
  double alpha_fast;
};

template <class Mixin, class RandomNumberGenerator = typename std::mt19937>
struct AdaptiveGeneration : public Mixin {
  // Heuristic that adds random samples to the set of particles to allow the
  // algorithm to recover in case all particles close to the correct pose
  // have been discarded.
  // It determines how many random particles to add by monitoring
  // the estimate accuracy averaging the importance factor of the particles.
  // The estimate takes into consideration the divergence between
  // the short-term and the long-term average.
  // See 'Probabilistics Robotics, Chapter 8.3.3'.
 public:
  using param_type = AdaptiveGenerationParam;

  template <class... Args>
  explicit AdaptiveGeneration(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), slow_filter_{parameters.alpha_slow}, fast_filter_{parameters.alpha_fast} {}

  template <class Range>
  [[nodiscard]] auto generate_samples_from(Range&& particles) {
    auto&& weights = views::weights(particles);
    double total_weight = std::reduce(std::begin(weights), std::end(weights), 0.);
    double average_weight = total_weight / static_cast<double>(weights.size());
    double random_state_probability = std::max(0., 1. - fast_filter_(average_weight) / slow_filter_(average_weight));

    if (random_state_probability > 0.) {
      // Reset averages to avoid spiraling off into complete randomness.
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
  AverageFilter slow_filter_;
  AverageFilter fast_filter_;
};

struct FixedResamplingParam {
  std::size_t max_samples;
};

template <class Mixin>
struct FixedResampling : public Mixin {
 public:
  using param_type = FixedResamplingParam;

  template <class... Args>
  explicit FixedResampling(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), parameters_{parameters} {}

  [[nodiscard]] std::size_t min_samples() const { return parameters_.max_samples; }
  [[nodiscard]] std::size_t max_samples() const { return parameters_.max_samples; }

  [[nodiscard]] auto take_samples() const { return ranges::views::take_exactly(parameters_.max_samples); }

 private:
  param_type parameters_;
};

struct KldResamplingParam {
  std::size_t min_samples;
  std::size_t max_samples;
  double spatial_resolution;
  double kld_epsilon;
  double kld_z;
};

template <class Mixin>
struct KldResampling : public Mixin {
 public:
  using param_type = KldResamplingParam;

  template <class... Args>
  explicit KldResampling(const param_type& parameters, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), parameters_{parameters} {}

  [[nodiscard]] std::size_t min_samples() const { return parameters_.min_samples; }
  [[nodiscard]] std::size_t max_samples() const { return parameters_.max_samples; }

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
