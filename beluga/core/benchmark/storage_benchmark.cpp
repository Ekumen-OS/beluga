#include <benchmark/benchmark.h>

#include <beluga/storage.h>

namespace {

struct State {
  double x = 0.;
  double y = 0.;
  double theta = 0.;
};

constexpr std::size_t kParticleCount = 1'000'000;

double update_weight(const State& state) {
  return state.x * state.y * state.theta;
}

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
      weights[i] = update_weight(states[i]);
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
      auto& particle = particles[i];
      particle.weight = update_weight(particle.state);
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
    std::transform(std::begin(states), std::end(states), std::begin(weights), update_weight);
  }
}

BENCHMARK(BM_UpdateWeights_Baseline_StructureOfArrays)->MinWarmUpTime(1);
BENCHMARK(BM_UpdateWeights<beluga::core::storage::StructureOfArrays<State>>)->MinWarmUpTime(1);
BENCHMARK(BM_UpdateWeights_Baseline_ArrayOfStructures)->MinWarmUpTime(1);
BENCHMARK(BM_UpdateWeights<beluga::core::storage::ArrayOfStructures<State>>)->MinWarmUpTime(1);

void BM_Resample_PushBack_Baseline_StructureOfArrays(benchmark::State& state) {
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

void BM_Resample_Assign_Baseline_StructureOfArrays(benchmark::State& state) {
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

  new_states.resize(size);
  new_weights.resize(size);
  new_clusters.resize(size);

  for (auto _ : state) {
    for (std::size_t i = 0; i < size; ++i) {
      new_states[i] = State{};
      new_weights[i] = 0;
      new_clusters[i] = 0;
    }
  }
}

void BM_Resample_PushBack_Baseline_ArrayOfStructures(benchmark::State& state) {
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

void BM_Resample_Assign_Baseline_ArrayOfStructures(benchmark::State& state) {
  struct Particle {
    State state;
    double weight;
    std::size_t cluster;
  };

  std::vector<Particle> particles;
  particles.resize(kParticleCount);

  auto size = particles.size();
  std::vector<Particle> new_particles;
  new_particles.resize(size);

  for (auto _ : state) {
    new_particles.clear();
    for (std::size_t i = 0; i < size; ++i) {
      new_particles[i] = Particle{State{}, 0, 0};
    }
  }
}

template <class StoragePolicy>
void BM_Resample_PushBack(benchmark::State& state) {
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

template <class StoragePolicy>
void BM_Resample_Assign(benchmark::State& state) {
  using Container = typename StoragePolicy::container_type;
  using Particle = typename StoragePolicy::particle_type;
  Container container;
  container.resize(kParticleCount);
  Container new_container;
  new_container.resize(container.size());
  for (auto _ : state) {
    auto&& states = StoragePolicy::state_view(container);
    auto&& particles = StoragePolicy::particle_view(new_container);
    std::transform(std::begin(states), std::end(states), std::begin(particles), [](const State&) {
      return Particle{State{}, 0, 0};
    });
  }
}

BENCHMARK(BM_Resample_PushBack_Baseline_StructureOfArrays)->MinWarmUpTime(1);
BENCHMARK(BM_Resample_PushBack<beluga::core::storage::StructureOfArrays<State>>)->MinWarmUpTime(1);
BENCHMARK(BM_Resample_PushBack_Baseline_ArrayOfStructures)->MinWarmUpTime(1);
BENCHMARK(BM_Resample_PushBack<beluga::core::storage::ArrayOfStructures<State>>)->MinWarmUpTime(1);

BENCHMARK(BM_Resample_Assign_Baseline_StructureOfArrays)->MinWarmUpTime(1);
BENCHMARK(BM_Resample_Assign<beluga::core::storage::StructureOfArrays<State>>)->MinWarmUpTime(1);
BENCHMARK(BM_Resample_Assign_Baseline_ArrayOfStructures)->MinWarmUpTime(1);
BENCHMARK(BM_Resample_Assign<beluga::core::storage::ArrayOfStructures<State>>)->MinWarmUpTime(1);

}  // namespace
