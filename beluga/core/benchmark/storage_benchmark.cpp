#include <benchmark/benchmark.h>

#include <beluga/storage.h>

namespace {

struct State {
  double x = 0.;
  double y = 0.;
  double theta = 0.;
};

constexpr std::size_t kParticleCount = 1'000'000;

void BM_UpdateWeights_Baseline_StructureOfArrays(benchmark::State& state) {
  std::vector<State> states;
  std::vector<double> weights;
  std::vector<std::size_t> clusters;
  states.resize(kParticleCount);
  weights.resize(kParticleCount);
  clusters.resize(kParticleCount);
  auto size = states.size();
  for (auto _ : state) {
    for (std::size_t i = 0; i < size; ++i) {
      weights[i] = 1.;
    }
  }
}

void BM_UpdateWeights_Baseline_ArrayOfStructures(benchmark::State& state) {
  struct Particle {
    State state;
    double weight;
    std::size_t cluster;
  };

  std::vector<Particle> particles;
  particles.resize(kParticleCount);
  auto size = particles.size();
  for (auto _ : state) {
    for (std::size_t i = 0; i < size; ++i) {
      particles[i].weight = 1.;
    }
  }
}

template <class StoragePolicy>
void BM_UpdateWeights(benchmark::State& state) {
  using Container = typename StoragePolicy::container_type;
  Container container;
  container.resize(kParticleCount);
  for (auto _ : state) {
    auto&& states = StoragePolicy::state_view(container);
    auto&& weights = StoragePolicy::weight_view(container);
    std::transform(std::begin(states), std::end(states), std::begin(weights), [](const State) { return 1.; });
  }
}

BENCHMARK(BM_UpdateWeights_Baseline_StructureOfArrays)->MinWarmUpTime(1);
BENCHMARK(BM_UpdateWeights<beluga::core::storage::StructureOfArrays<State>>)->MinWarmUpTime(1);
BENCHMARK(BM_UpdateWeights_Baseline_ArrayOfStructures)->MinWarmUpTime(1);
BENCHMARK(BM_UpdateWeights<beluga::core::storage::ArrayOfStructures<State>>)->MinWarmUpTime(1);

void BM_Resample_Baseline_StructureOfArrays(benchmark::State& state) {
  std::vector<State> states;
  std::vector<double> weights;
  std::vector<std::size_t> clusters;
  states.resize(kParticleCount);
  weights.resize(kParticleCount);
  clusters.resize(kParticleCount);

  auto size = states.size();
  std::vector<State> new_states;
  std::vector<double> new_weights;
  std::vector<std::size_t> new_clusters;
  new_states.reserve(size);
  new_weights.reserve(size);
  new_clusters.reserve(size);

  for (auto _ : state) {
    new_states.clear();
    new_weights.clear();
    new_clusters.clear();
    for (std::size_t i = 0; i < size; ++i) {
      new_states.push_back(State{});
      new_weights.push_back(0);
      new_clusters.push_back(0);
    }
  }
}

void BM_Resample_Baseline_ArrayOfStructures(benchmark::State& state) {
  struct Particle {
    State state;
    double weight;
    std::size_t cluster;
  };

  std::vector<Particle> particles;
  particles.resize(kParticleCount);

  auto size = particles.size();
  std::vector<Particle> new_particles;
  new_particles.reserve(size);
  for (auto _ : state) {
    new_particles.clear();
    for (std::size_t i = 0; i < size; ++i) {
      new_particles.push_back(Particle{State{}, 0, 0});
    }
  }
}

template <class StoragePolicy>
void BM_Resample(benchmark::State& state) {
  using Container = typename StoragePolicy::container_type;
  using Particle = typename StoragePolicy::particle_type;
  Container container;
  container.resize(kParticleCount);
  Container new_container;
  new_container.reserve(container.size());
  for (auto _ : state) {
    auto&& states = StoragePolicy::state_view(container);
    new_container.clear();
    std::transform(std::begin(states), std::end(states), std::back_inserter(new_container), [](const State&) {
      return Particle{State{}, 0, 0};
    });
  }
}

BENCHMARK(BM_Resample_Baseline_StructureOfArrays)->MinWarmUpTime(1);
BENCHMARK(BM_Resample<beluga::core::storage::StructureOfArrays<State>>)->MinWarmUpTime(1);
BENCHMARK(BM_Resample_Baseline_ArrayOfStructures)->MinWarmUpTime(1);
BENCHMARK(BM_Resample<beluga::core::storage::ArrayOfStructures<State>>)->MinWarmUpTime(1);

}  // namespace
