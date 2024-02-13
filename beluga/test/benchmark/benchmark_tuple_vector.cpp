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

#include <beluga/containers/tuple_vector.hpp>
#include <beluga/type_traits.hpp>
#include <beluga/views/particles.hpp>

#include <range/v3/algorithm/transform.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>

namespace {

constexpr std::size_t kParticleCount = 1'000'000;

struct State {
  double x = 0.;
  double y = 0.;
  double theta = 0.;
};

using Particle = std::tuple<State, beluga::Weight, beluga::Cluster>;
using StructureOfArrays = beluga::TupleVector<std::tuple<State, beluga::Weight, beluga::Cluster>>;
using ArrayOfStructures = beluga::Vector<std::tuple<State, beluga::Weight, beluga::Cluster>>;

struct Arrays {
  std::vector<State> states;
  std::vector<double> weights;
  std::vector<std::size_t> clusters;

  [[nodiscard]] std::size_t size() const noexcept { return states.size(); }

  void clear() noexcept {
    states.clear();
    weights.clear();
    clusters.clear();
  }

  void reserve(std::size_t new_cap) {
    states.reserve(new_cap);
    weights.reserve(new_cap);
    clusters.reserve(new_cap);
  }

  void resize(std::size_t count) {
    states.resize(count);
    weights.resize(count);
    clusters.resize(count);
  }
};

double update_weight(const State& state) {
  return state.x * state.y * state.theta;
}

void BM_Update_Baseline_StructureOfArrays(benchmark::State& state) {
  Arrays arrays;
  arrays.resize(kParticleCount);
  for (auto _ : state) {
    for (std::size_t i = 0; i < kParticleCount; ++i) {
      arrays.weights[i] = update_weight(arrays.states[i]);
    }
  }
}

void BM_Update_Baseline_ArrayOfStructures(benchmark::State& state) {
  std::vector<Particle> particles;
  particles.resize(kParticleCount);
  for (auto _ : state) {
    for (std::size_t i = 0; i < kParticleCount; ++i) {
      auto&& particle = particles[i];
      std::get<1>(particle) = update_weight(std::get<0>(particle));
    }
  }
}

template <class Container>
void BM_Update(benchmark::State& state) {
  auto container = Container{};
  container.resize(kParticleCount);
  for (auto _ : state) {
    auto weights = beluga::views::weights(container);
    ranges::transform(beluga::views::states(container), ranges::begin(weights), update_weight);
  }
}

BENCHMARK(BM_Update_Baseline_StructureOfArrays);
BENCHMARK_TEMPLATE(BM_Update, StructureOfArrays);
BENCHMARK(BM_Update_Baseline_ArrayOfStructures);
BENCHMARK_TEMPLATE(BM_Update, ArrayOfStructures);

void BM_PushBack_Baseline_StructureOfArrays(benchmark::State& state) {
  Arrays arrays;
  arrays.resize(kParticleCount);
  Arrays new_arrays;
  new_arrays.reserve(kParticleCount);
  for (auto _ : state) {
    new_arrays.clear();
    for (std::size_t i = 0; i < kParticleCount; ++i) {
      new_arrays.states.push_back(State{});
      new_arrays.weights.push_back(0);
      new_arrays.clusters.push_back(0);
    }
  }
}

void BM_PushBack_Baseline_ArrayOfStructures(benchmark::State& state) {
  std::vector<Particle> particles;
  particles.resize(kParticleCount);
  std::vector<Particle> new_particles;
  new_particles.reserve(kParticleCount);
  for (auto _ : state) {
    new_particles.clear();
    for (std::size_t i = 0; i < kParticleCount; ++i) {
      new_particles.emplace_back(std::get<0>(particles[i]), 0, 0);
    }
  }
}

template <class Container>
void BM_PushBack(benchmark::State& state) {
  auto container = Container{};
  container.resize(kParticleCount);
  auto new_container = Container{};
  new_container.reserve(kParticleCount);
  for (auto _ : state) {
    new_container.clear();
    ranges::transform(beluga::views::states(container), ranges::back_inserter(new_container), [](const State& state) {
      return std::make_tuple(state, 0, 0);
    });
  }
}

BENCHMARK(BM_PushBack_Baseline_StructureOfArrays);
BENCHMARK_TEMPLATE(BM_PushBack, StructureOfArrays);
BENCHMARK(BM_PushBack_Baseline_ArrayOfStructures);
BENCHMARK_TEMPLATE(BM_PushBack, ArrayOfStructures);

void BM_Assign_Baseline_ArrayOfStructures(benchmark::State& state) {
  std::vector<Particle> particles;
  particles.resize(kParticleCount);
  std::vector<Particle> new_particles;
  new_particles.resize(kParticleCount);
  for (auto _ : state) {
    for (std::size_t i = 0; i < kParticleCount; ++i) {
      new_particles[i] = Particle{std::get<0>(particles[i]), 0, 0};
    }
  }
}

void BM_Assign_Baseline_StructureOfArrays(benchmark::State& state) {
  Arrays arrays;
  arrays.resize(kParticleCount);
  Arrays new_arrays;
  new_arrays.resize(kParticleCount);
  for (auto _ : state) {
    for (std::size_t i = 0; i < kParticleCount; ++i) {
      new_arrays.states[i] = arrays.states[i];
      new_arrays.weights[i] = 0;
      new_arrays.clusters[i] = 0;
    }
  }
}

template <class Container>
void BM_Transform(benchmark::State& state) {
  auto container = Container{};
  container.resize(kParticleCount);
  auto new_container = Container{};
  new_container.resize(kParticleCount);
  for (auto _ : state) {
    ranges::transform(beluga::views::states(container), ranges::begin(new_container), [](const State& state) {
      return std::make_tuple(state, 0, 0);
    });
  }
}

template <class Container>
void BM_AssignFromSized(benchmark::State& state) {
  auto container = Container{};
  container.resize(kParticleCount);
  auto new_container = Container{};
  for (auto _ : state) {
    new_container.assign_range(
        beluga::views::states(container) |  //
        ranges::views::transform([](const State& state) { return std::make_tuple(state, 0, 0); }));
  }
}

template <class Container>
void BM_AssignFromNonSized(benchmark::State& state) {
  auto container = Container{};
  container.resize(kParticleCount);
  auto new_container = Container{};
  for (auto _ : state) {
    new_container.assign_range(
        beluga::views::states(container) |                  //
        ranges::views::filter([](auto) { return true; }) |  //
        ranges::views::transform([](const State& state) { return std::make_tuple(state, 0, 0); }));
  }
}

template <class Container>
void BM_AssignFromNonSizedReserved(benchmark::State& state) {
  auto container = Container{};
  container.resize(kParticleCount);
  auto new_container = Container{};
  new_container.reserve(kParticleCount);
  for (auto _ : state) {
    new_container.assign_range(
        beluga::views::states(container) |                  //
        ranges::views::filter([](auto) { return true; }) |  //
        ranges::views::transform([](const State& state) { return std::make_tuple(state, 0, 0); }));
  }
}

BENCHMARK(BM_Assign_Baseline_StructureOfArrays);
BENCHMARK_TEMPLATE(BM_AssignFromSized, StructureOfArrays);
BENCHMARK_TEMPLATE(BM_AssignFromNonSized, StructureOfArrays);
BENCHMARK_TEMPLATE(BM_AssignFromNonSizedReserved, StructureOfArrays);
BENCHMARK_TEMPLATE(BM_Transform, StructureOfArrays);
BENCHMARK(BM_Assign_Baseline_ArrayOfStructures);
BENCHMARK_TEMPLATE(BM_Transform, ArrayOfStructures);

}  // namespace
