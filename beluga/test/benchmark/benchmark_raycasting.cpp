// Copyright 2023 Ekumen, Inc.
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

#include <cstddef>
#include <initializer_list>
#include <vector>

#include <Eigen/Core>

#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

#include "beluga/algorithm/raycasting.hpp"
#include "beluga/algorithm/raycasting/bresenham.hpp"
#include "beluga/sensor/data/occupancy_grid.hpp"
#include "beluga/test/raycasting.hpp"
#include "beluga/test/static_occupancy_grid.hpp"

namespace {

void BM_Bresenham2i(benchmark::State& state, beluga::Bresenham2i::Variant variant) {
  const auto n = state.range(0);
  const auto algorithm = beluga::Bresenham2i{variant};
  for (auto _ : state) {
    Eigen::Vector2i storage;
    for (const auto& cell : algorithm({0, 0}, {n, n})) {
      benchmark::DoNotOptimize(storage = cell);
    }
  }
  state.SetComplexityN(n);
}

BENCHMARK_CAPTURE(BM_Bresenham2i, Standard, beluga::Bresenham2i::kStandard)
    ->RangeMultiplier(2)
    ->Range(128, 4096)
    ->Complexity();

BENCHMARK_CAPTURE(BM_Bresenham2i, Modified, beluga::Bresenham2i::kModified)
    ->RangeMultiplier(2)
    ->Range(128, 4096)
    ->Complexity();

using beluga::testing::StaticOccupancyGrid;

template <std::size_t Rows, std::size_t Cols, class T = bool>
class BaselineGrid : public beluga::BaseOccupancyGrid2<BaselineGrid<Rows, Cols, T>> {
 public:
  explicit BaselineGrid(std::initializer_list<T>, double resolution) : resolution_{resolution} {
    std::fill(std::begin(data()), std::end(data()), false);
  }

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] auto& data() { return grid_; }
  [[nodiscard]] const auto& data() const { return grid_; }
  [[nodiscard]] std::size_t size() const { return grid_.size(); }

  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }
  [[nodiscard]] double resolution() const { return resolution_; }

  [[nodiscard]] auto value_traits() const { return beluga::testing::ValueTraits<T>{}; }

 private:
  double resolution_;
  Sophus::SE2d origin_;
  std::array<T, Rows * Cols> grid_;
  static constexpr std::size_t kWidth = Cols;
  static constexpr std::size_t kHeight = Rows;
};

const auto kBearingAngles = std::array{
    0.,                                    // Horizontal
    Sophus::Constants<double>::pi() / 2.,  // Vertical
    Sophus::Constants<double>::pi() / 4.,  // Diagonal
};

const auto kBearingLabels = std::array{
    "Horizontal",
    "Vertical",
    "Diagonal",
};

template <template <std::size_t, std::size_t, class T = bool> class Grid>
void BM_RayCasting2d_BaselineRaycast(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;

  const auto bearing_index = static_cast<std::size_t>(state.range(0));
  const auto n = static_cast<int>(state.range(1));
  Grid<1280, 1280> grid{{}, kResolution};
  grid.data()[grid.index_at(n, n)] = true;
  grid.data()[grid.index_at(0, n)] = true;
  grid.data()[grid.index_at(n, 0)] = true;

  const auto source_pose = Eigen::Vector2d{0., 0.};
  const auto beam_bearing = Sophus::SO2d{kBearingAngles.at(bearing_index)};

  const auto source = grid.cell_near(source_pose);
  const auto target = grid.cell_near(source_pose + kMaxRange * beam_bearing.unit_complex());

  for (auto _ : state) {
    benchmark::DoNotOptimize(beluga::testing::raycast(grid, source, target));
  }
  state.SetComplexityN(n);
  state.SetLabel(kBearingLabels.at(bearing_index));
}

BENCHMARK_TEMPLATE(BM_RayCasting2d_BaselineRaycast, BaselineGrid)
    ->Args({0, 128})
    ->Args({0, 256})
    ->Args({0, 512})
    ->Args({0, 1024})
    ->Args({1, 128})
    ->Args({1, 256})
    ->Args({1, 512})
    ->Args({1, 1024})
    ->Args({2, 128})
    ->Args({2, 256})
    ->Args({2, 512})
    ->Args({2, 1024})
    ->Complexity();

BENCHMARK_TEMPLATE(BM_RayCasting2d_BaselineRaycast, StaticOccupancyGrid)
    ->Args({0, 128})
    ->Args({0, 256})
    ->Args({0, 512})
    ->Args({0, 1024})
    ->Args({1, 128})
    ->Args({1, 256})
    ->Args({1, 512})
    ->Args({1, 1024})
    ->Args({2, 128})
    ->Args({2, 256})
    ->Args({2, 512})
    ->Args({2, 1024})
    ->Complexity();

template <template <std::size_t, std::size_t, class T = bool> class Grid>
void BM_RayCasting2d(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;

  const auto bearing_index = static_cast<std::size_t>(state.range(0));
  const auto n = static_cast<int>(state.range(1));
  auto grid = Grid<1280, 1280>{{}, kResolution};
  grid.data()[grid.index_at(n, n)] = true;
  grid.data()[grid.index_at(0, n)] = true;
  grid.data()[grid.index_at(n, 0)] = true;

  const auto source_pose = Sophus::SE2d{0., Eigen::Vector2d{0., 0.}};
  const auto beam_bearing = Sophus::SO2d{kBearingAngles.at(bearing_index)};
  const auto beam = beluga::Ray2d{grid, source_pose, kMaxRange};
  for (auto _ : state) {
    benchmark::DoNotOptimize(beam.cast(beam_bearing));
  }
  state.SetComplexityN(n);
  state.SetLabel(kBearingLabels.at(bearing_index));
}

BENCHMARK_TEMPLATE(BM_RayCasting2d, BaselineGrid)
    ->Args({0, 128})
    ->Args({0, 256})
    ->Args({0, 512})
    ->Args({0, 1024})
    ->Args({1, 128})
    ->Args({1, 256})
    ->Args({1, 512})
    ->Args({1, 1024})
    ->Args({2, 128})
    ->Args({2, 256})
    ->Args({2, 512})
    ->Args({2, 1024})
    ->Complexity();

BENCHMARK_TEMPLATE(BM_RayCasting2d, StaticOccupancyGrid)
    ->Args({0, 128})
    ->Args({0, 256})
    ->Args({0, 512})
    ->Args({0, 1024})
    ->Args({1, 128})
    ->Args({1, 256})
    ->Args({1, 512})
    ->Args({1, 1024})
    ->Args({2, 128})
    ->Args({2, 256})
    ->Args({2, 512})
    ->Args({2, 1024})
    ->Complexity();

}  // namespace
