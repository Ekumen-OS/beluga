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

#include <numeric>
#include <random>
#include <utility>

#include <benchmark/benchmark.h>

#include <beluga/algorithm/raycasting.hpp>
#include <beluga/test/raycasting.hpp>
#include <beluga/test/static_occupancy_grid.hpp>

#include <range/v3/range/conversion.hpp>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

using beluga::testing::PlainGridStorage;
using beluga::testing::StaticOccupancyGrid;

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

template <std::size_t Rows, std::size_t Cols>
class BaselineGrid : public beluga::BaseOccupancyGrid2<BaselineGrid<Rows, Cols>> {
 public:
  struct ValueTraits {
    [[nodiscard]] bool is_free(bool value) const { return !value; }
    [[nodiscard]] bool is_unknown(bool) const { return false; }
    [[nodiscard]] bool is_occupied(bool value) const { return value; }
  };

  explicit BaselineGrid(PlainGridStorage<Rows, Cols>&& grid, double resolution)
      : grid_{std::move(grid)}, resolution_{resolution} {}

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] auto& data() { return grid_.data(); }
  [[nodiscard]] const auto& data() const { return grid_.data(); }
  [[nodiscard]] std::size_t size() const { return grid_.size(); }

  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }
  [[nodiscard]] double resolution() const { return resolution_; }

  [[nodiscard]] auto value_traits() const { return ValueTraits{}; }

 private:
  PlainGridStorage<Rows, Cols> grid_;
  double resolution_;
  Sophus::SE2d origin_;
  static constexpr std::size_t kWidth = Cols;
  static constexpr std::size_t kHeight = Rows;
};

enum class RaycastBearing { kHorizontal, kVertical, kDiagonal };

constexpr auto bearingOrdinal(RaycastBearing bearing) {
  return static_cast<int>(bearing);
}

constexpr auto bearingAngle(RaycastBearing bearing) {
  switch (bearing) {
    case RaycastBearing::kHorizontal:
      return 0.0;
    case RaycastBearing::kVertical:
      return Sophus::Constants<double>::pi() / 2.0;
    case RaycastBearing::kDiagonal:
      return Sophus::Constants<double>::pi() / 4.0;
  }
  throw std::runtime_error{"Invalid bearing"};
}

constexpr auto bearingLabels(RaycastBearing bearing) {
  switch (bearing) {
    case RaycastBearing::kHorizontal:
      return "Horizontal";
    case RaycastBearing::kVertical:
      return "Vertical";
    case RaycastBearing::kDiagonal:
      return "Diagonal";
  }
  throw std::runtime_error{"Invalid bearing"};
}

template <template <std::size_t, std::size_t> class Grid>
void BM_RayCasting2d_BaselineRaycast(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;
  constexpr auto kGridSize = 1280;

  const auto bearing_index = static_cast<RaycastBearing>(state.range(0));
  const auto n = static_cast<int>(state.range(1));

  auto grid_storage = PlainGridStorage<kGridSize, kGridSize>{};
  grid_storage.cell(n, n) = true;
  grid_storage.cell(0, n) = true;
  grid_storage.cell(n, 0) = true;
  Grid grid{std::move(grid_storage), kResolution};

  const auto source_pose = Eigen::Vector2d{0., 0.};
  const auto beam_bearing = Sophus::SO2d{bearingAngle(bearing_index)};

  const auto source = grid.cell_near(source_pose);
  const auto target = grid.cell_near(source_pose + kMaxRange * beam_bearing.unit_complex());

  for (auto _ : state) {
    benchmark::DoNotOptimize(beluga::testing::raycast(grid, source, target));
  }
  state.SetComplexityN(n);
  state.SetLabel(bearingLabels(bearing_index));
}

BENCHMARK_TEMPLATE(BM_RayCasting2d_BaselineRaycast, BaselineGrid)
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 256})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 512})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 1024})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 256})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 512})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 1024})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 256})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 512})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 1024})
    ->Complexity();

BENCHMARK_TEMPLATE(BM_RayCasting2d_BaselineRaycast, StaticOccupancyGrid)
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 256})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 512})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 1024})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 128})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 256})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 512})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 1024})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 256})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 512})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 1024})
    ->Complexity();

template <template <std::size_t, std::size_t> class Grid>
void BM_RayCasting2d(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;
  constexpr auto kGridSize = 1280;

  const auto bearing_index = static_cast<RaycastBearing>(state.range(0));
  const auto n = static_cast<int>(state.range(1));

  auto grid_storage = PlainGridStorage<kGridSize, kGridSize>{};
  grid_storage.cell(n, n) = true;
  grid_storage.cell(0, n) = true;
  grid_storage.cell(n, 0) = true;
  auto grid = Grid{std::move(grid_storage), kResolution};

  const auto source_pose = Sophus::SE2d{0., Eigen::Vector2d{0., 0.}};
  const auto beam_bearing = Sophus::SO2d{bearingAngle(bearing_index)};
  const auto beam = beluga::Ray2d{grid, source_pose, kMaxRange};
  for (auto _ : state) {
    benchmark::DoNotOptimize(beam.cast(beam_bearing));
  }
  state.SetComplexityN(n);
  state.SetLabel(bearingLabels(bearing_index));
}

BENCHMARK_TEMPLATE(BM_RayCasting2d, BaselineGrid)
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 256})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 512})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 1024})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 128})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 256})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 512})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 1024})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 256})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 512})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 1024})
    ->Complexity();

BENCHMARK_TEMPLATE(BM_RayCasting2d, StaticOccupancyGrid)
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 256})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 512})
    ->Args({bearingOrdinal(RaycastBearing::kHorizontal), 1024})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 128})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 256})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 512})
    ->Args({bearingOrdinal(RaycastBearing::kVertical), 1024})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 128})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 256})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 512})
    ->Args({bearingOrdinal(RaycastBearing::kDiagonal), 1024})
    ->Complexity();

template <template <std::size_t, std::size_t> class Grid>
void BM_RayCasting2d_GridCacheFriendlyness(benchmark::State& state) {
  // This benchmark is intended to measure the effecto of the direction of traversal of the grid due to cache locality
  // effects
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;
  constexpr auto kGridSize = 4100;

  // whether to apply randomness to the source pose or not. If not we'll hit the same cache lines over and over again
  const auto apply_randomness = static_cast<bool>(state.range(0));
  // raycast distance to obstacle
  const auto n = static_cast<int>(state.range(1));
  // direction of the raycast (horizontal or vertical only)
  const auto bearing_index = static_cast<RaycastBearing>(state.range(2));

  // draw an obstacle at the limits of the grid
  auto grid_storage = PlainGridStorage<kGridSize, kGridSize>{};
  for (int i = 0; i < kGridSize; ++i) {
    grid_storage.cell(kGridSize - 1, i) = true;
    grid_storage.cell(i, kGridSize - 1) = true;
  }
  auto grid = Grid<kGridSize, kGridSize>{std::move(grid_storage), kResolution};

  const auto end_of_grid_coord = static_cast<double>(kGridSize - 1) * kResolution;
  const auto distance_to_obstacle = static_cast<double>(n) * kResolution;

  std::default_random_engine fixed_seed_generator;
  std::uniform_real_distribution<double> uniform_dist(0.0, end_of_grid_coord);

  const auto beam_bearing = Sophus::SO2d{bearingAngle(bearing_index)};
  auto source_pose = Sophus::SE2d{};

  const auto flush_cache = [] {
    static std::array<int, 32 * 1024 * 1024 / sizeof(int)> v;
    return std::accumulate(v.begin(), v.end(), 0);
  };
  benchmark::DoNotOptimize(flush_cache());

  for (auto _ : state) {
    const auto offset = apply_randomness ? uniform_dist(fixed_seed_generator) : 0.0;
    if (bearing_index == RaycastBearing::kHorizontal) {
      source_pose = Sophus::SE2d{0., Eigen::Vector2d{end_of_grid_coord - distance_to_obstacle, offset}};
    } else if (bearing_index == RaycastBearing::kVertical) {
      source_pose = Sophus::SE2d{0., Eigen::Vector2d{offset, end_of_grid_coord - distance_to_obstacle}};
    }
    const auto beam = beluga::Ray2d{grid, source_pose, kMaxRange};
    benchmark::DoNotOptimize(beam.cast(beam_bearing));
  }
  state.SetComplexityN(n);
  state.SetLabel(std::string{} + bearingLabels(bearing_index) + "/" + (apply_randomness ? "random" : "deterministic"));
}

BENCHMARK_TEMPLATE(BM_RayCasting2d_GridCacheFriendlyness, BaselineGrid)
    ->ArgsProduct({
        {false, true},
        {128, 256, 512, 1024, 2048, 4096},
        {bearingOrdinal(RaycastBearing::kHorizontal), bearingOrdinal(RaycastBearing::kVertical)},
    })
    ->Complexity();

BENCHMARK_TEMPLATE(BM_RayCasting2d_GridCacheFriendlyness, StaticOccupancyGrid)
    ->ArgsProduct({
        {false, true},
        {128, 256, 512, 1024, 2048, 4096},
        {bearingOrdinal(RaycastBearing::kHorizontal), bearingOrdinal(RaycastBearing::kVertical)},
    })
    ->Complexity();

}  // namespace
