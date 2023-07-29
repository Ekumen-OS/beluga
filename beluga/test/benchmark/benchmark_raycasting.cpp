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
#include <beluga/sensor/data/cache_friendly_grid_storage.hpp>
#include <beluga/sensor/data/cache_friendly_grid_storage_2.hpp>
#include <beluga/sensor/data/cache_friendly_grid_storage_3.hpp>
#include <beluga/sensor/data/cache_friendly_grid_storage_4.hpp>
#include <beluga/sensor/data/dense_grid2_mixin.hpp>
#include <beluga/sensor/data/linear_grid_storage.hpp>
#include <beluga/sensor/data/regular_grid2_mixin.hpp>

#include <beluga/test/raycasting.hpp>
#include <beluga/test/static_occupancy_grid.hpp>

#include <range/v3/range/conversion.hpp>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

#include <ciabatta/ciabatta.hpp>

using beluga::CacheFriendlyGridStorage;
using beluga::CacheFriendlyGridStorage2;
using beluga::CacheFriendlyGridStorage3;
using beluga::CacheFriendlyGridStorage4;
using beluga::LinearGridStorage;

using beluga::testing::GridWithConfigurableStorage;
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

template <class Grid>
void BM_RayCasting2d_AMCLStraightMotion(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;
  constexpr auto kGridSize = 1280;

  const auto bearing_index = static_cast<RaycastBearing>(state.range(0));
  const auto n = static_cast<int>(state.range(1));

  auto grid_storage = typename Grid::MapStorage(kGridSize, kGridSize);
  grid_storage.cell(n, n) = true;
  grid_storage.cell(0, n) = true;
  grid_storage.cell(n, 0) = true;
  auto grid = Grid(std::move(grid_storage), kResolution, Sophus::SE2d{});

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

BENCHMARK_TEMPLATE(BM_RayCasting2d_AMCLStraightMotion, StaticOccupancyGrid)
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

template <class Grid>
void BM_RayCasting2d_BelugaStraightMotion(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;
  constexpr auto kGridSize = 1280;

  const auto bearing_index = static_cast<RaycastBearing>(state.range(0));
  const auto n = static_cast<int>(state.range(1));

  auto grid_storage = typename Grid::MapStorage(kGridSize, kGridSize);
  grid_storage.cell(n, n) = true;
  grid_storage.cell(0, n) = true;
  grid_storage.cell(n, 0) = true;
  auto grid = Grid(std::move(grid_storage), kResolution, Sophus::SE2d{});

  const auto source_pose = Sophus::SE2d{0., Eigen::Vector2d{0., 0.}};
  const auto beam_bearing = Sophus::SO2d{bearingAngle(bearing_index)};
  const auto beam = beluga::Ray2d{grid, source_pose, kMaxRange};
  for (auto _ : state) {
    benchmark::DoNotOptimize(beam.cast(beam_bearing));
  }
  state.SetComplexityN(n);
  state.SetLabel(bearingLabels(bearing_index));
}

BENCHMARK_TEMPLATE(BM_RayCasting2d_BelugaStraightMotion, StaticOccupancyGrid)
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

enum class WalkDirection { kWestEast, kEastWest, kNorthSouth, kSouthNorth };

template <class Storage>
void BM_GridStorage_StraightMotion(benchmark::State& state) {
  // this benchmark measures the performance of different grid storage alternatives when
  // walking the grid in vertical vs horizontal patterns.
  using CellType = typename Storage::cell_type;
  constexpr auto kShortSideLenght = 2048;  // this needs to be larger than a cache line

  const auto walk_direction = static_cast<WalkDirection>(state.range(0));
  const std::size_t bytes_to_cover = state.range(1) * 1024;  // the value is assumed in kilobytes
  const auto cells_to_cover = bytes_to_cover / sizeof(CellType);

  std::string direction_label;
  std::size_t grid_width{1}, grid_height{1};
  int init_x = 0;
  int init_y = 0;
  int increment_x = 0;
  int increment_y = 0;

  switch (walk_direction) {
    case WalkDirection::kWestEast:
      direction_label = "WestEast";
      grid_width = cells_to_cover;
      grid_height = kShortSideLenght;
      init_x = 0;
      init_y = static_cast<int>(grid_height / 2);
      increment_x = 1;
      increment_y = 0;
      break;
    case WalkDirection::kEastWest:
      direction_label = "EastWest";
      grid_width = cells_to_cover;
      grid_height = kShortSideLenght;
      init_x = static_cast<int>(grid_width - 1);
      init_y = static_cast<int>(grid_height / 2);
      increment_x = -1;
      increment_y = 0;
      break;
    case WalkDirection::kNorthSouth:
      direction_label = "NorthSouth";
      grid_width = kShortSideLenght;
      grid_height = cells_to_cover;
      init_x = static_cast<int>(grid_width / 2);
      init_y = 0;
      increment_x = 0;
      increment_y = 1;
      break;
    case WalkDirection::kSouthNorth:
      direction_label = "SouthNorth";
      grid_width = kShortSideLenght;
      grid_height = cells_to_cover;
      init_x = static_cast<int>(grid_width / 2);
      init_y = static_cast<int>(grid_height - 1);
      increment_x = 0;
      increment_y = -1;
      break;
  }

  auto storage = Storage(grid_width, grid_height);

  for (auto _ : state) {
    auto x = init_x;
    auto y = init_y;
    for (std::size_t i = 0; i < cells_to_cover; ++i) {
      benchmark::DoNotOptimize(storage.cell(x, y));
      x += increment_x;
      y += increment_y;
    }
  }

  state.SetLabel(direction_label);
}

BENCHMARK_TEMPLATE(BM_GridStorage_StraightMotion, LinearGridStorage<bool>)
    ->Args({static_cast<int>(WalkDirection::kWestEast), 1})  // smaller than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 1})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 1})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 1})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 16})  // halfway through L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 16})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 16})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 16})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 64})  // larger than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 64})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 64})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 64})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 512})  // larger than L2 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 512})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 512})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 512});

BENCHMARK_TEMPLATE(BM_GridStorage_StraightMotion, CacheFriendlyGridStorage<bool, 64>)
    ->Args({static_cast<int>(WalkDirection::kWestEast), 1})  // smaller than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 1})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 1})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 1})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 16})  // halfway through L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 16})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 16})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 16})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 64})  // larger than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 64})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 64})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 64})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 512})  // larger than L2 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 512})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 512})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 512});

BENCHMARK_TEMPLATE(BM_GridStorage_StraightMotion, CacheFriendlyGridStorage2<bool, 64>)
    ->Args({static_cast<int>(WalkDirection::kWestEast), 1})  // smaller than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 1})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 1})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 1})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 16})  // halfway through L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 16})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 16})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 16})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 64})  // larger than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 64})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 64})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 64})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 512})  // larger than L2 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 512})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 512})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 512});

BENCHMARK_TEMPLATE(BM_GridStorage_StraightMotion, CacheFriendlyGridStorage3<bool>)
    ->Args({static_cast<int>(WalkDirection::kWestEast), 1})  // smaller than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 1})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 1})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 1})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 16})  // halfway through L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 16})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 16})
    ->Args(
        {static_cast<int>(WalkDirection::kSouthNorth),
         16});  // can't afford the massive grid needed for the rest beyond this size

BENCHMARK_TEMPLATE(BM_GridStorage_StraightMotion, CacheFriendlyGridStorage4<bool, 64>)
    ->Args({static_cast<int>(WalkDirection::kWestEast), 1})  // smaller than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 1})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 1})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 1})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 16})  // halfway through L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 16})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 16})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 16})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 64})  // larger than L1 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 64})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 64})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 64})
    ->Args({static_cast<int>(WalkDirection::kWestEast), 512})  // larger than L2 cache
    ->Args({static_cast<int>(WalkDirection::kEastWest), 512})
    ->Args({static_cast<int>(WalkDirection::kNorthSouth), 512})
    ->Args({static_cast<int>(WalkDirection::kSouthNorth), 512});

template <class Storage>
void BM_GridStorage_RadialMotion(benchmark::State& state) {
  // this benchmark is intended to compare the behavior of the grid storage alternatives when
  // walking a ray radially from a central point through a grid storage, with a finite number of rays.
  // This measures only the performance of the storage.
  using CellType = typename Storage::cell_type;

  constexpr auto kRadialRays = 24;

  const auto active_set_size = state.range(0) * 1024;  // the value is assumed in kilobytes
  const auto bytes_to_cover_per_ray = active_set_size / kRadialRays;
  const auto radius = bytes_to_cover_per_ray / sizeof(CellType);
  const auto kGridSide = 2 * radius + 1;  // intentionally odd to have a defined center

  const int xc = static_cast<int>(radius);
  const int yc = static_cast<int>(radius);

  auto storage = Storage(kGridSide, kGridSide);

  std::vector<std::pair<double, double>> directions;

  for (int i = 0; i < kRadialRays; ++i) {
    const auto rad = static_cast<double>(i) * 2. * Sophus::Constants<double>::pi() / static_cast<double>(kRadialRays);
    directions.emplace_back(std::cos(rad), std::sin(rad));
  }

  std::size_t direction_index = 0;
  for (auto _ : state) {
    const auto [cx, cy] = directions[direction_index];
    direction_index = (direction_index + 1) % directions.size();

    for (std::size_t l = 0; l < radius; ++l) {
      const auto x = xc + static_cast<int>(static_cast<double>(l) * cx);
      const auto y = yc + static_cast<int>(static_cast<double>(l) * cy);
      benchmark::DoNotOptimize(storage.cell(x, y));
    }
  }
}

BENCHMARK_TEMPLATE(BM_GridStorage_RadialMotion, LinearGridStorage<bool>)
    ->Args({1})
    ->Args({8})
    ->Args({16})
    ->Args({32})  // full L1 cache usage
    ->Args({64})
    ->Args({128})
    ->Args({256})
    ->Args({512})
    ->Args({1024});

BENCHMARK_TEMPLATE(BM_GridStorage_RadialMotion, CacheFriendlyGridStorage<bool, 64>)
    ->Args({1})
    ->Args({8})
    ->Args({16})
    ->Args({32})  // full L1 cache usage
    ->Args({64})
    ->Args({128})
    ->Args({256})
    ->Args({512})
    ->Args({1024});

BENCHMARK_TEMPLATE(BM_GridStorage_RadialMotion, CacheFriendlyGridStorage2<bool, 64>)
    ->Args({1})
    ->Args({8})
    ->Args({16})
    ->Args({32})  // full L1 cache usage
    ->Args({64})
    ->Args({128})
    ->Args({256})
    ->Args({512})
    ->Args({1024});

BENCHMARK_TEMPLATE(BM_GridStorage_RadialMotion, CacheFriendlyGridStorage3<bool>)
    ->Args({1})
    ->Args({8})
    ->Args({16})
    ->Args({32})  // full L1 cache usage
    ->Args({64})
    ->Args({128})
    ->Args({256})
    ->Args({512})
    ->Args({1024});

BENCHMARK_TEMPLATE(BM_GridStorage_RadialMotion, CacheFriendlyGridStorage4<bool, 64>)
    ->Args({1})
    ->Args({8})
    ->Args({16})
    ->Args({32})  // full L1 cache usage
    ->Args({64})
    ->Args({128})
    ->Args({256})
    ->Args({512})
    ->Args({1024});

template <class Grid>
void BM_RayCasting2d_StorageImpact_RadialMotion(benchmark::State& state) {
  // this benchmark is intended to compare the behavior of the grid storage alternatives when
  // performing raytracing radially from a central point for a finite number of rays in clowise order.
  // This measures the impact of the storage on the tracking algorithm.
  constexpr auto kRadialRays = 192;
  constexpr auto kResolution = 0.05;

  const auto radius_meters = static_cast<double>(state.range(0));
  const auto radius_pixels = static_cast<std::size_t>(radius_meters / kResolution);

  const auto kGridSide = 2 * radius_pixels + 1;  // intentionally odd to have a well-defined center

  const auto metric_center_x = radius_meters;
  const auto metric_center_y = radius_meters;

  auto grid_storage = typename Grid::MapStorage(kGridSide, kGridSide);
  // everything beyond a radius from the center is an obstacle
  [[maybe_unused]] auto squared = [](std::size_t x) { return x * x; };
  for (std::size_t pixel_x_coord = 0; pixel_x_coord < kGridSide; ++pixel_x_coord) {
    for (std::size_t pixel_y_coord = 0; pixel_y_coord < kGridSide; ++pixel_y_coord) {
      if (squared(pixel_x_coord - radius_pixels) + squared(pixel_y_coord - radius_pixels) > squared(radius_pixels)) {
        grid_storage.cell(static_cast<int>(pixel_x_coord), static_cast<int>(pixel_y_coord)) = true;
      } else {
        grid_storage.cell(static_cast<int>(pixel_x_coord), static_cast<int>(pixel_y_coord)) = false;
      }
    }
  }
  auto grid = Grid(std::move(grid_storage), kResolution, Sophus::SE2d{});

  std::vector<Sophus::SO2d> precalculated_bearings;
  for (int i = 0; i < kRadialRays; ++i) {
    const auto rad = static_cast<double>(i) * 2. * Sophus::Constants<double>::pi() / static_cast<double>(kRadialRays);
    precalculated_bearings.emplace_back(Sophus::SO2d{rad});
  }

  const auto source_pose = Sophus::SE2d{0., Eigen::Vector2d{metric_center_x, metric_center_y}};
  const auto beam = beluga::Ray2d{grid, source_pose, radius_meters + 10.0};

  std::size_t direction_index = 0;
  for (auto _ : state) {
    const auto& beam_bearing = precalculated_bearings[direction_index];
    direction_index = (direction_index + 1) % precalculated_bearings.size();
    benchmark::DoNotOptimize(beam.cast(beam_bearing));
  }
}

BENCHMARK_TEMPLATE(BM_RayCasting2d_StorageImpact_RadialMotion, GridWithConfigurableStorage<LinearGridStorage<bool>>)
    ->Args({20})  // radius in meters
    ->Args({50})
    ->Args({100})
    ->Args({200});

BENCHMARK_TEMPLATE(
    BM_RayCasting2d_StorageImpact_RadialMotion,
    GridWithConfigurableStorage<CacheFriendlyGridStorage<bool, 64>>)
    ->Args({20})  // radius in meters
    ->Args({50})
    ->Args({100})
    ->Args({200});

BENCHMARK_TEMPLATE(
    BM_RayCasting2d_StorageImpact_RadialMotion,
    GridWithConfigurableStorage<CacheFriendlyGridStorage2<bool, 64>>)
    ->Args({20})  // radius in meters
    ->Args({50})
    ->Args({100})
    ->Args({200});

BENCHMARK_TEMPLATE(
    BM_RayCasting2d_StorageImpact_RadialMotion,
    GridWithConfigurableStorage<CacheFriendlyGridStorage3<bool>>)
    ->Args({20})  // radius in meters
    ->Args({50})
    ->Args({100})
    ->Args({200});

BENCHMARK_TEMPLATE(
    BM_RayCasting2d_StorageImpact_RadialMotion,
    GridWithConfigurableStorage<CacheFriendlyGridStorage4<bool, 64>>)
    ->Args({20})  // radius in meters
    ->Args({50})
    ->Args({100})
    ->Args({200});

}  // namespace
