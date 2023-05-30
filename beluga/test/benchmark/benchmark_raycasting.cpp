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

#include <vector>

#include <benchmark/benchmark.h>

#include <beluga/algorithm/raycasting.hpp>
#include <beluga/test/raycasting.hpp>
#include <beluga/test/static_occupancy_grid.hpp>

#include <range/v3/range/conversion.hpp>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

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

template <std::size_t Rows, std::size_t Cols>
class BaselineGrid : public beluga::BaseOccupancyGrid2<BaselineGrid<Rows, Cols>> {
 public:
  struct ValueTraits {
    [[nodiscard]] bool is_free(bool value) const { return !value; }
    [[nodiscard]] bool is_unknown(bool) const { return false; }
    [[nodiscard]] bool is_occupied(bool value) const { return value; }
  };

  explicit BaselineGrid(std::initializer_list<bool>, double resolution) : resolution_{resolution} {
    std::fill(std::begin(data()), std::end(data()), false);
  }

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] auto& data() { return grid_; }
  [[nodiscard]] const auto& data() const { return grid_; }
  [[nodiscard]] std::size_t size() const { return grid_.size(); }

  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }
  [[nodiscard]] double resolution() const { return resolution_; }

  [[nodiscard]] auto value_traits() const { return ValueTraits{}; }

 private:
  double resolution_;
  Sophus::SE2d origin_;
  std::array<bool, Rows * Cols> grid_;
  static constexpr std::size_t kWidth = Cols;
  static constexpr std::size_t kHeight = Rows;
};

template <template <std::size_t, std::size_t> class Grid>
void BM_RayCasting2d_BaselineRaycast(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;

  Grid<1280, 1280> map{{}, kResolution};

  const auto n = static_cast<int>(state.range(0));
  map.data()[map.index_at(n, n)] = true;

  const auto source_pose = Eigen::Vector2d{1., 1.};
  const auto beam_bearing = Sophus::SO2d{Sophus::Constants<double>::pi() / 4.};

  const auto source = map.cell_near(source_pose);
  const auto target = map.cell_near(source_pose + kMaxRange * beam_bearing.unit_complex());

  for (auto _ : state) {
    benchmark::DoNotOptimize(beluga::testing::raycast(map, source, target));
  }

  state.SetComplexityN(n);
}

BENCHMARK_TEMPLATE(BM_RayCasting2d_BaselineRaycast, BaselineGrid)->RangeMultiplier(2)->Range(128, 1024)->Complexity();
BENCHMARK_TEMPLATE(BM_RayCasting2d_BaselineRaycast, StaticOccupancyGrid)
    ->RangeMultiplier(2)
    ->Range(128, 1024)
    ->Complexity();

template <template <std::size_t, std::size_t> class Grid>
void BM_RayCasting2d(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;

  const auto n = static_cast<int>(state.range(0));
  auto grid = Grid<1280, 1280>{{}, kResolution};
  grid.data()[grid.index_at(n, n)] = true;

  const auto source_pose = Sophus::SE2d{0., Eigen::Vector2d{1., 1.}};
  const auto beam_bearing = Sophus::SO2d{Sophus::Constants<double>::pi() / 4.};
  const auto beam = beluga::Ray2d{grid, source_pose, kMaxRange};
  for (auto _ : state) {
    benchmark::DoNotOptimize(beam.cast(beam_bearing));
  }
  state.SetComplexityN(n);
}

BENCHMARK_TEMPLATE(BM_RayCasting2d, BaselineGrid)->RangeMultiplier(2)->Range(128, 1024)->Complexity();
BENCHMARK_TEMPLATE(BM_RayCasting2d, StaticOccupancyGrid)->RangeMultiplier(2)->Range(128, 1024)->Complexity();

}  // namespace
