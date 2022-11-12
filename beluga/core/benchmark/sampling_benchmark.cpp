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
struct beluga::spatial_hash<State> {
  std::size_t operator()(const State& state, double resolution = 1.) const {
    const auto pair = std::make_tuple(state.x, state.y);
    return beluga::spatial_hash<std::decay_t<decltype(pair)>>{}(pair, resolution);
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

  std::size_t min_samples = particle_count;

  for (auto _ : state) {
    auto&& samples = ranges::views::generate(beluga::random_sample(
                         beluga::views::all(container), beluga::views::weights(container), generator)) |
                     ranges::views::take_exactly(min_samples) | ranges::views::common;
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

  std::size_t min_samples = 0;
  std::size_t max_samples = particle_count;
  double resolution = 1.;
  double kld_epsilon = 0.05;
  double kld_z = 1.28155156327703;  // P = 0.9

  for (auto _ : state) {
    auto&& samples = ranges::views::generate(beluga::random_sample(
                         beluga::views::all(container), beluga::views::weights(container), generator)) |
                     ranges::views::transform(beluga::set_cluster(resolution)) |
                     ranges::views::take_while(
                         beluga::kld_condition(min_samples, kld_epsilon, kld_z), beluga::cluster<const Particle&>) |
                     ranges::views::take(max_samples) | ranges::views::common;
    auto first = std::begin(beluga::views::all(new_container));
    auto last = std::copy(std::begin(samples), std::end(samples), first);
    state.counters["SampleSize"] = std::distance(first, last);
    state.counters["Percentage"] = static_cast<double>(std::distance(first, last)) / particle_count;
  }
}

BENCHMARK(BM_AdaptiveResample)->MinWarmUpTime(1)->RangeMultiplier(2)->Range(128, 1'000'000)->Complexity();

}  // namespace
