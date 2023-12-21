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

#include <benchmark/benchmark.h>

#include <beluga/algorithm/sampling.hpp>
#include <beluga/tuple_vector.hpp>
#include <beluga/type_traits.hpp>
#include <beluga/views/particles.hpp>
#include <range/v3/view.hpp>

namespace {

struct State {
  double x = 0.;
  double y = 0.;
  double theta = 0.;
};

}  // namespace

template <>
struct beluga::spatial_hash<State> {
  std::size_t operator()(const State& state) const {
    const auto tuple = std::make_tuple(state.x, state.y);
    return beluga::spatial_hash<std::decay_t<decltype(tuple)>>{std::array{1., 1.}}(tuple);
  }
};

namespace {

using Particle = std::tuple<State, beluga::Weight, beluga::Cluster>;
using Container = beluga::TupleOfVectors<State, beluga::Weight, beluga::Cluster>;

void BM_FixedResample(benchmark::State& state) {
  const auto particle_count = state.range(0);
  state.SetComplexityN(particle_count);
  const auto container_size = static_cast<std::size_t>(particle_count);

  auto container = Container{container_size};
  auto new_container = Container{container_size};
  auto generator = std::mt19937{std::random_device()()};

  for (auto&& [state, weight, cluster] : container) {
    weight = 1.;
  }

  for (auto _ : state) {
    auto&& samples = ranges::views::generate(beluga::make_multinomial_sampler(
                         ranges::views::all(container), beluga::views::weights(container), generator)) |
                     ranges::views::take_exactly(container_size) | ranges::views::common;
    auto first = std::begin(container);
    auto last = std::copy(std::begin(samples), std::end(samples), first);
    state.counters["SampleSize"] = static_cast<double>(std::distance(first, last));
  }
}

BENCHMARK(BM_FixedResample)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

void BM_AdaptiveResample(benchmark::State& state) {
  const auto particle_count = state.range(0);
  state.SetComplexityN(particle_count);
  const auto container_size = static_cast<std::size_t>(particle_count);

  auto container = Container{container_size};
  auto new_container = Container{container_size};
  auto generator = std::mt19937{std::random_device()()};

  double step = 0;
  int i = 0;
  for (auto&& [state, weight, cluster] : container) {
    weight = 1.;
    state.x = step;
    if (i++ % 2 == 0) {
      step += 0.05;
    }
  }

  std::size_t min_samples = 0;
  std::size_t max_samples = container_size;
  auto hasher = beluga::spatial_hash<State>{};
  double kld_epsilon = 0.05;
  double kld_z = 3.;

  for (auto _ : state) {
    auto&& samples =
        ranges::views::generate(beluga::make_multinomial_sampler(
            ranges::views::all(container), beluga::views::weights(container), generator)) |
        ranges::views::transform(beluga::make_clusterization_function(hasher)) |
        ranges::views::take_while(beluga::kld_condition(min_samples, kld_epsilon, kld_z), beluga::cluster) |
        ranges::views::take(container_size) | ranges::views::common;
    auto first = std::begin(container);
    auto last = std::copy(std::begin(samples), std::end(samples), first);
    state.counters["SampleSize"] = static_cast<double>(std::distance(first, last));
    state.counters["Percentage"] = static_cast<double>(std::distance(first, last)) / static_cast<double>(max_samples);
  }
}

BENCHMARK(BM_AdaptiveResample)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

}  // namespace
