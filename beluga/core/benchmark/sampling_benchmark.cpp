#include <benchmark/benchmark.h>

#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>

#include <beluga/algorithm/sampling.h>
#include <beluga/tuple_vector.h>
#include <beluga/type_traits.h>

namespace {

struct State {
  double x = 0.;
  double y = 0.;
  double theta = 0.;
};

}  // namespace

template <>
struct beluga::voxel_hash<State> {
  std::size_t operator()(const State& state, double voxel_size = 1.) const {
    const auto pair = std::make_tuple(state.x, state.y);
    return beluga::voxel_hash<std::decay_t<decltype(pair)>>{}(pair, voxel_size);
  }
};

namespace {

using Particle = std::tuple<State, double, std::size_t>;
using Container = beluga::TupleVector<Particle>;

void BM_FixedResample(benchmark::State& state) {
  const std::size_t particle_count = state.range(0);
  state.SetComplexityN(particle_count);

  auto container = Container{particle_count};
  auto new_container = Container{particle_count};
  auto generator = std::mt19937{std::random_device()()};

  for (auto&& [state, weight, cluster] : beluga::views::all(container)) {
    weight = 1.;
  }

  struct Params {
    std::size_t min_samples;
  };

  auto parameters = Params{.min_samples = static_cast<std::size_t>(particle_count)};

  for (auto _ : state) {
    auto&& samples = beluga::views::fixed_resample(container, generator, parameters) | ranges::views::common;
    auto first = std::begin(beluga::views::all(new_container));
    auto last = std::copy(std::begin(samples), std::end(samples), first);
    state.counters["SampleSize"] = std::distance(first, last);
  }
}

BENCHMARK(BM_FixedResample)->MinWarmUpTime(1)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

void BM_AdaptiveResample(benchmark::State& state) {
  const std::size_t particle_count = state.range(0);
  state.SetComplexityN(particle_count);

  auto container = Container{particle_count};
  auto new_container = Container{particle_count};
  auto generator = std::mt19937{std::random_device()()};

  double step = 0;
  int i = 0;
  for (auto&& [state, weight, cluster] : beluga::views::all(container)) {
    weight = 1.;
    state.x = step;
    if (i++ % 2 == 0) {
      step += 0.05;
    }
  }

  struct Params {
    std::size_t min_samples;
    std::size_t max_samples;
    double voxel_size;
    double kld_epsilon;
    double kld_upper_quantile;
  };

  auto parameters = Params{
      .min_samples = 0,
      .max_samples = particle_count,
      .voxel_size = 1.,
      .kld_epsilon = 0.05,
      .kld_upper_quantile = 0.95};

  for (auto _ : state) {
    auto&& samples = beluga::views::adaptive_resample(container, generator, parameters) | ranges::views::common;
    auto first = std::begin(beluga::views::all(new_container));
    auto last = std::copy(std::begin(samples), std::end(samples), first);
    state.counters["SampleSize"] = std::distance(first, last);
    state.counters["Percentage"] = static_cast<double>(std::distance(first, last)) / particle_count;
  }
}

BENCHMARK(BM_AdaptiveResample)->MinWarmUpTime(1)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

}  // namespace
