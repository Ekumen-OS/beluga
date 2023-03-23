// Copyright 2022 Ekumen, Inc.
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

#include <beluga/algorithm/spatial_hash.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>

namespace {

void BM_Hashing(benchmark::State& state) {
  using Tuple = std::tuple<double, double, double>;
  constexpr std::array kClusteringResolution{1., 1., 1.};
  auto hasher = beluga::spatial_hash<Tuple>{kClusteringResolution};

  const auto count = state.range(0);
  state.SetComplexityN(count);

  for (auto _ : state) {
    auto hashes = ranges::views::generate([]() { return std::make_tuple(1., 2., 3.); }) |
                  ranges::views::transform([hasher](const auto& tuple) { return hasher(tuple); }) |
                  ranges::views::take_exactly(count);

    auto first = std::begin(hashes);
    auto last = std::end(hashes);
    std::size_t value;
    while (first != last) {
      benchmark::DoNotOptimize(value = *first++);
    }
  }
}

BENCHMARK(BM_Hashing)->RangeMultiplier(2)->Range(100'000, 1'000'000)->Complexity();

}  // namespace
