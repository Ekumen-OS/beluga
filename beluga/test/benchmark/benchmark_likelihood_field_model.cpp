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

#include <benchmark/benchmark.h>

#include <array>
#include <cstddef>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

#include "beluga/sensor/likelihood_field_model.hpp"
#include "beluga/test/static_occupancy_grid.hpp"

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

template <std::size_t N>
void BM_MakeLikelihoodField(benchmark::State& state) {
  using Grid = beluga::testing::StaticOccupancyGrid<N, N>;
  // A closed boundary plus scattered 3x3 obstacles, at 5 cm per cell.
  auto cells = std::make_unique<std::array<bool, N * N>>();
  for (std::size_t i = 0; i < N; ++i) {
    (*cells)[i] = (*cells)[(N - 1) * N + i] = (*cells)[i * N] = (*cells)[i * N + N - 1] = true;
  }
  auto engine = std::mt19937{42};
  auto cell = std::uniform_int_distribution<std::size_t>{1, N - 3};
  for (std::size_t k = 0; k < N / 4; ++k) {
    const std::size_t x = cell(engine);
    const std::size_t y = cell(engine);
    for (std::size_t dy = 0; dy < 3; ++dy) {
      for (std::size_t dx = 0; dx < 3; ++dx) {
        (*cells)[(y + dy) * N + x + dx] = true;
      }
    }
  }
  const auto grid = std::make_unique<Grid>(*cells, 0.05);
  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  for (auto _ : state) {
    auto model = std::make_unique<beluga::LikelihoodFieldModel<Grid>>(params, *grid);
    benchmark::DoNotOptimize(model->likelihood_field().data().data());
  }
}

BENCHMARK(BM_MakeLikelihoodField<200>)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_MakeLikelihoodField<500>)->Unit(benchmark::kMillisecond);

BENCHMARK(BM_PointTransform_Baseline)->RangeMultiplier(2)->Range(128, 1024)->Complexity();
BENCHMARK(BM_PointTransform_EigenSophus)->RangeMultiplier(2)->Range(128, 1024)->Complexity();

}  // namespace
