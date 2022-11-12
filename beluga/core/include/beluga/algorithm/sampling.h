#pragma once

#include <functional>
#include <limits>
#include <random>
#include <unordered_set>

#include <absl/random/discrete_distribution.h>
#include <beluga/spatial_hash.h>
#include <beluga/type_traits.h>

namespace beluga {

template <class Function1, class Function2, class RandomNumberGenerator>
auto random_select(Function1&& first, Function2&& second, RandomNumberGenerator& generator, double probability) {
  return [&, distribution = std::bernoulli_distribution{probability}]() mutable {
    return distribution(generator) ? first() : second();
  };
}

template <class Range, class Weights, class RandomNumberGenerator>
auto random_sample(const Range& samples, const Weights& weights, RandomNumberGenerator& generator) {
  return [&generator, first = std::begin(samples),
          distribution = absl::discrete_distribution<std::size_t>{std::begin(weights), std::end(weights)}]() mutable {
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

inline auto kld_condition(std::size_t min, double epsilon, double z = 1.28155156327703) {
  // Compute minimum number of samples based on a Kullback-Leibler distance epsilon
  // between the maximum likelihood estimate and the true distribution.
  // Z is the upper_standard normal quantile for (1 - P), where P is the probability
  // that the error in the estimated distribution will be less than epsilon.
  // Here are some examples:
  //     P = 0.90 -> Z = 1.28155156327703
  //     P = 0.95 -> Z = 1.64485362793663
  //     P = 0.99 -> Z = 2.32634787735669
  auto target_size = [two_epsilon = 2 * epsilon, z](std::size_t k) {
    if (k <= 2U) {
      return std::numeric_limits<std::size_t>::max();
    }
    double common = 2. / (9 * (k - 1));
    double base = 1. - common + std::sqrt(common) * z;
    double result = ((k - 1) / two_epsilon) * base * base * base;
    return static_cast<std::size_t>(std::ceil(result));
  };

  return [=, count = 0ULL, buckets = std::unordered_set<std::size_t>{}](std::size_t hash) mutable {
    count++;
    buckets.insert(hash);
    return count < min || count < target_size(buckets.size());
  };
}

}  // namespace beluga
