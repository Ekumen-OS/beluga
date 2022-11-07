#pragma once

#include <functional>
#include <random>
#include <unordered_set>

#include <absl/random/discrete_distribution.h>
#include <range/v3/view.hpp>

#include <beluga/type_traits.h>
#include <beluga/voxel_hash.h>

namespace beluga {

template <class Function1, class Function2, class RandomNumberGenerator>
auto random_select(Function1&& first, Function2&& second, RandomNumberGenerator& generator, double probability) {
  return [&, probability, distribution = std::uniform_real_distribution{}]() mutable {
    return distribution(generator) < probability ? first() : second();
  };
}

template <class Range, class Weights, class RandomNumberGenerator>
auto random_sample(const Range& samples, const Weights& weights, RandomNumberGenerator& generator) {
  return [&generator, first = std::begin(samples),
          distribution = absl::discrete_distribution<std::size_t>{std::begin(weights), std::end(weights)}]() mutable {
    return *(first + distribution(generator));
  };
}

namespace detail {

inline auto set_cluster(double voxel_size) {
  return [voxel_size](auto&& particle) {
    using state_type = std::decay_t<decltype(state(particle))>;
    auto new_particle = particle;
    cluster(new_particle) = voxel_hash<state_type>{}(state(particle), voxel_size);
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

}  // namespace detail

namespace views {

template <class Container, class RandomNumberGenerator, class Params>
auto fixed_resample(Container&& particles, RandomNumberGenerator& random_number_generator, const Params& params) {
  return ranges::views::generate(random_sample(all(particles), weights(particles), random_number_generator)) |
         ranges::views::take_exactly(params.min_samples);
}

template <class Container, class RandomStateGenerator, class RandomNumberGenerator, class Params>
auto fixed_resample(
    Container&& particles,
    RandomStateGenerator& random_state_generator,
    RandomNumberGenerator& random_number_generator,
    const Params& params) {
  using particle_type = typename std::decay_t<Container>::value_type;
  return ranges::views::generate(random_select(
             random_state_generator, random_sample(states(particles), weights(particles), random_number_generator),
             random_number_generator, params.random_state_probability)) |
         ranges::views::transform(make_from_state<particle_type>) | ranges::views::take_exactly(params.min_samples);
}

template <class Container, class RandomNumberGenerator, class Params>
auto adaptive_resample(Container&& particles, RandomNumberGenerator& random_number_generator, const Params& params) {
  using particle_type = typename std::decay_t<Container>::value_type;
  return ranges::views::generate(random_sample(all(particles), weights(particles), random_number_generator)) |
         ranges::views::transform(detail::set_cluster(params.voxel_size)) |
         ranges::views::take_while(
             detail::kld_condition(params.min_samples, params.kld_epsilon, params.kld_upper_quantile),
             cluster<const particle_type&>) |
         ranges::views::take(params.max_samples);
}

template <class Container, class RandomStateGenerator, class RandomNumberGenerator, class Params>
auto adaptive_resample(
    Container&& particles,
    RandomStateGenerator& random_state_generator,
    RandomNumberGenerator& random_number_generator,
    const Params& params) {
  using particle_type = typename std::decay_t<Container>::value_type;
  return ranges::views::generate(random_select(
             random_state_generator, random_sample(states(particles), weights(particles), random_number_generator),
             random_number_generator, params.random_state_probability)) |
         ranges::views::transform(make_from_state<particle_type>) |
         ranges::views::transform(detail::set_cluster(params.voxel_size)) |
         ranges::views::take_while(
             detail::kld_condition(params.min_samples, params.kld_epsilon, params.kld_upper_quantile),
             cluster<const particle_type&>) |
         ranges::views::take(params.max_samples);
}

}  // namespace views

}  // namespace beluga
