// Copyright 2022-2024 Ekumen, Inc.
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

#include <array>
#include <cstddef>
#include <tuple>
#include <type_traits>

#include <range/v3/algorithm/copy.hpp>
#include <range/v3/range/access.hpp>
#include <range/v3/range/primitives.hpp>
#include <range/v3/view/subrange.hpp>
#include <range/v3/view/take_exactly.hpp>

#include "beluga/algorithm/spatial_hash.hpp"
#include "beluga/containers/tuple_vector.hpp"
#include "beluga/primitives.hpp"
#include "beluga/views/sample.hpp"
#include "beluga/views/take_while_kld.hpp"

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

using Container = beluga::TupleVector<std::tuple<State, beluga::Weight>>;
using Particle = typename Container::value_type;

void BM_FixedResample(benchmark::State& state) {
  const auto particle_count = state.range(0);
  state.SetComplexityN(particle_count);
  const auto container_size = static_cast<std::size_t>(particle_count);

  auto container = Container{container_size};
  auto new_container = Container{container_size};

  for (auto&& [state, weight] : container) {
    weight = 1.;
  }

  for (auto _ : state) {
    auto samples = container |              //
                   beluga::views::sample |  //
                   ranges::views::take_exactly(container_size);
    auto first = ranges::begin(new_container);
    auto last = ranges::copy(samples, first).out;
    auto result = ranges::make_subrange(first, last);
    state.counters["SampleSize"] = static_cast<double>(ranges::size(result));
  }
}

BENCHMARK(BM_FixedResample)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

void BM_AdaptiveResample(benchmark::State& state) {
  const auto particle_count = state.range(0);
  state.SetComplexityN(particle_count);
  const auto container_size = static_cast<std::size_t>(particle_count);

  auto container = Container{container_size};
  auto new_container = Container{container_size};

  double step = 0;
  int i = 0;
  for (auto&& [state, weight] : container) {
    weight = 1.;
    state.x = step;
    if (i++ % 2 == 0) {
      step += 0.05;
    }
  }

  const std::size_t min_samples = 0;
  const std::size_t max_samples = container_size;
  const double kld_epsilon = 0.05;
  const double kld_z = 3.;

  for (auto _ : state) {
    auto samples =
        container |              //
        beluga::views::sample |  //
        beluga::views::take_while_kld(beluga::spatial_hash<State>{}, min_samples, max_samples, kld_epsilon, kld_z);
    auto first = ranges::begin(new_container);
    auto last = ranges::copy(samples, first).out;
    auto result = ranges::make_subrange(first, last);
    state.counters["SampleSize"] = static_cast<double>(ranges::size(result));
    state.counters["Percentage"] = static_cast<double>(ranges::size(result)) / static_cast<double>(max_samples);
  }
}

BENCHMARK(BM_AdaptiveResample)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

}  // namespace
