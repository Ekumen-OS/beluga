#include <benchmark/benchmark.h>

#include <range/v3/view.hpp>

#include <beluga/voxel_hash.h>

namespace {

void BM_Hashing(benchmark::State& state) {
  using Tuple = std::tuple<double, double, double>;

  const auto count = state.range(0);
  state.SetComplexityN(count);

  for (auto _ : state) {
    auto hashes = ranges::views::generate([]() { return std::make_tuple(1., 2., 3.); }) |
                  ranges::views::transform([](const auto& tuple) { return beluga::voxel_hash<Tuple>{}(tuple); }) |
                  ranges::views::take_exactly(count);

    auto first = std::begin(hashes);
    auto last = std::end(hashes);
    while (first != last) {
      benchmark::DoNotOptimize(++first);
    }
  }
}

BENCHMARK(BM_Hashing)->MinWarmUpTime(1)->RangeMultiplier(2)->Range(100'000, 1'000'000)->Complexity();

}  // namespace
