// Copyright 2025 Ekumen, Inc.
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
#include <random>
#include <tuple>
#include <type_traits>

#include <range/v3/algorithm/copy.hpp>
#include <range/v3/range/access.hpp>
#include <range/v3/range/primitives.hpp>
#include <range/v3/view/subrange.hpp>
#include <range/v3/view/take_exactly.hpp>

#include "beluga/containers/tuple_vector.hpp"
#include "beluga/primitives.hpp"
#include "beluga/views/low_variance_sample.hpp"
#include "beluga/views/sample.hpp"

namespace {

struct State {
  double x = 0.;
  double y = 0.;
  double theta = 0.;
};

using Container = beluga::TupleVector<std::tuple<State, beluga::Weight>>;
using Particle = typename Container::value_type;

// Helper function to create a container with uniform weights
Container create_uniform_container(std::size_t size) {
  auto container = Container{size};
  for (auto&& [state, weight] : container) {
    weight = 1.0;
  }
  return container;
}

// Helper function to create a container with varied weights
Container create_weighted_container(std::size_t size) {
  auto container = Container{size};
  std::mt19937 gen(42);  // Fixed seed for reproducibility
  std::uniform_real_distribution<double> dist(0.1, 2.0);

  for (auto&& [state, weight] : container) {
    weight = dist(gen);
  }
  return container;
}

// Helper function to create a container with exponentially distributed weights
Container create_exponential_container(std::size_t size) {
  auto container = Container{size};
  std::mt19937 gen(42);  // Fixed seed for reproducibility
  std::exponential_distribution<double> dist(1.0);

  for (auto&& [state, weight] : container) {
    weight = dist(gen) + 0.01;  // Add small offset to avoid zero weights
  }
  return container;
}

}  // namespace

// Benchmark low variance sampling with uniform weights
void BM_LowVarianceSample_Uniform(benchmark::State& state) {
  const auto particle_count = state.range(0);
  state.SetComplexityN(particle_count);
  const auto container_size = static_cast<std::size_t>(particle_count);

  auto container = create_uniform_container(container_size);
  auto new_container = Container{container_size};

  for (auto _ : state) {
    auto samples = container |                           //
                   beluga::views::low_variance_sample |  //
                   ranges::views::take_exactly(container_size);
    auto first = ranges::begin(new_container);
    auto last = ranges::copy(samples, first).out;
    auto result = ranges::make_subrange(first, last);
    benchmark::DoNotOptimize(result);
  }
}

BENCHMARK(BM_LowVarianceSample_Uniform)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

// Benchmark low variance sampling with varied weights
void BM_LowVarianceSample_Weighted(benchmark::State& state) {
  const auto particle_count = state.range(0);
  state.SetComplexityN(particle_count);
  const auto container_size = static_cast<std::size_t>(particle_count);

  auto container = create_weighted_container(container_size);
  auto new_container = Container{container_size};

  for (auto _ : state) {
    auto samples = container |                           //
                   beluga::views::low_variance_sample |  //
                   ranges::views::take_exactly(container_size);
    auto first = ranges::begin(new_container);
    auto last = ranges::copy(samples, first).out;
    auto result = ranges::make_subrange(first, last);
    benchmark::DoNotOptimize(result);
  }
}

BENCHMARK(BM_LowVarianceSample_Weighted)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

// Benchmark low variance sampling with exponentially distributed weights
void BM_LowVarianceSample_Exponential(benchmark::State& state) {
  const auto particle_count = state.range(0);
  state.SetComplexityN(particle_count);
  const auto container_size = static_cast<std::size_t>(particle_count);

  auto container = create_exponential_container(container_size);
  auto new_container = Container{container_size};

  for (auto _ : state) {
    auto samples = container |                           //
                   beluga::views::low_variance_sample |  //
                   ranges::views::take_exactly(container_size);
    auto first = ranges::begin(new_container);
    auto last = ranges::copy(samples, first).out;
    auto result = ranges::make_subrange(first, last);
    benchmark::DoNotOptimize(result);
  }
}

BENCHMARK(BM_LowVarianceSample_Exponential)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

// Benchmark sampling performance with different sample sizes (fixed population)
void BM_LowVarianceSample_VariableSampleSize(benchmark::State& state) {
  const auto sample_count = state.range(0);
  state.SetComplexityN(sample_count);
  const auto container_size = 10000;  // Fixed large population
  const auto sample_size = static_cast<std::size_t>(sample_count);

  auto container = create_weighted_container(container_size);
  auto new_container = Container{sample_size};

  for (auto _ : state) {
    auto samples = container |                           //
                   beluga::views::low_variance_sample |  //
                   ranges::views::take_exactly(sample_size);
    auto first = ranges::begin(new_container);
    auto last = ranges::copy(samples, first).out;
    auto result = ranges::make_subrange(first, last);
    benchmark::DoNotOptimize(result);
    state.counters["SampleSize"] = static_cast<double>(sample_size);
    state.counters["PopulationSize"] = static_cast<double>(container_size);
  }
}

BENCHMARK(BM_LowVarianceSample_VariableSampleSize)->RangeMultiplier(2)->Range(64, 8192)->Complexity();

// Benchmark memory access patterns by testing small vs large sample sizes
void BM_LowVarianceSample_MemoryPattern(benchmark::State& state) {
  const auto particle_count = state.range(0);
  state.SetComplexityN(particle_count);
  const auto container_size = static_cast<std::size_t>(particle_count);
  const auto sample_size = std::max(static_cast<std::size_t>(1), container_size / 10);  // Sample 10% of particles

  auto container = create_weighted_container(container_size);
  auto new_container = Container{sample_size};

  for (auto _ : state) {
    auto samples = container |                           //
                   beluga::views::low_variance_sample |  //
                   ranges::views::take_exactly(sample_size);
    auto first = ranges::begin(new_container);
    auto last = ranges::copy(samples, first).out;
    auto result = ranges::make_subrange(first, last);
    benchmark::DoNotOptimize(result);
    state.counters["SampleRatio"] = static_cast<double>(sample_size) / static_cast<double>(container_size);
  }
}

BENCHMARK(BM_LowVarianceSample_MemoryPattern)->RangeMultiplier(2)->Range(1000, 100'000)->Complexity();
