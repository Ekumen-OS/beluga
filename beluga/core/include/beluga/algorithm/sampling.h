#pragma once

#include <functional>
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

inline auto kld_condition(std::size_t min, double epsilon = 0.05, double upper_quantile = 0.95) {
  // Compute minimum number of samples based on a Kullback-Leibler distance epsilon
  // between the maximum likelihood estimate and the true distribution.
  auto target_size = [upper_quantile, epsilon](std::size_t k) {
    double common = 2. / (9 * (k - 1));
    double base = 1. - common - std::sqrt(common) * upper_quantile;
    double result = ((k - 1) / epsilon) * base * base * base;
    return static_cast<std::size_t>(std::ceil(result));
  };

  return [=, count = 0ULL, buckets = std::unordered_set<std::size_t>{}](std::size_t hash) mutable {
    count++;
    buckets.insert(hash);
    auto cluster_count = buckets.size();
    bool target_not_reached = cluster_count <= 2U ? true : count < target_size(cluster_count);
    return count < min || target_not_reached;
  };
}

}  // namespace beluga
