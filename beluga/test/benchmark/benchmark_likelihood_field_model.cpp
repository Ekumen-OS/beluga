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

#include <vector>

#include <benchmark/benchmark.h>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace {

constexpr std::size_t kNumParticles = 2'000;

void BM_PointTransform_Baseline(benchmark::State& state) {
  const auto count = state.range(0);
  state.SetComplexityN(count);
  auto size = static_cast<size_t>(count);
  const auto particles = std::vector<Sophus::SE2d>{kNumParticles};
  const auto points = std::vector<std::pair<double, double>>(size);
  const auto origin = Sophus::SE2d{};
  for (auto _ : state) {
    for (const auto& pose : particles) {
      const auto transform = origin * pose;
      const double x_offset = transform.translation().x();
      const double y_offset = transform.translation().y();
      const double cos_theta = transform.so2().unit_complex().x();
      const double sin_theta = transform.so2().unit_complex().y();
      for (const auto& point : points) {
        const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
        const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
        benchmark::DoNotOptimize(x + y);
      }
    }
  }
}

void BM_PointTransform_EigenSophus(benchmark::State& state) {
  const auto count = state.range(0);
  state.SetComplexityN(count);
  auto size = static_cast<size_t>(count);
  const auto particles = std::vector<Sophus::SE2d>{kNumParticles};
  const auto points = std::vector<Eigen::Vector2d>(size);
  const auto origin = Sophus::SE2d{};
  for (auto _ : state) {
    for (const auto& pose : particles) {
      const auto transform = origin * pose;
      for (const auto& point : points) {
        const auto result = transform * point;
        const auto x = result.x();
        const auto y = result.y();
        benchmark::DoNotOptimize(x + y);
      }
    }
  }
}

BENCHMARK(BM_PointTransform_Baseline)->RangeMultiplier(2)->Range(128, 1024)->Complexity();
BENCHMARK(BM_PointTransform_EigenSophus)->RangeMultiplier(2)->Range(128, 1024)->Complexity();

}  // namespace
