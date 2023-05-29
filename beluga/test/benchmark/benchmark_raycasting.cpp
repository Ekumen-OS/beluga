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
struct BaselineGrid {
 public:
  explicit BaselineGrid(std::initializer_list<bool>, double resolution) : resolution_{resolution} {
    std::fill(std::begin(data()), std::end(data()), false);
  }

  [[nodiscard]] auto origin() const { return Sophus::SE2d{}; }

  [[nodiscard]] const auto& data() const { return data_; }

  [[nodiscard]] auto& data() { return data_; }

  [[nodiscard]] double resolution() const { return resolution_; }

  [[nodiscard]] auto index_at(int i, int j) const {
    return static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * kWidth;
  }

  [[nodiscard]] bool contains(int i, int j) const {
    return (i >= 0) && (i < static_cast<int>(kWidth)) && (j >= 0) && (j < static_cast<int>(kHeight));
  }

  [[nodiscard]] bool contains(Eigen::Vector2i cell) const { return contains(cell.x(), cell.y()); }

  [[nodiscard]] auto data_at(std::size_t index) const {
    return index < Rows * Cols ? std::make_optional(data()[index]) : std::nullopt;
  }

  [[nodiscard]] auto data_at(int i, int j) const {
    return contains(i, j) ? std::make_optional(data()[index_at(i, j)]) : std::nullopt;
  }

  [[nodiscard]] auto data_at(const Eigen::Vector2i& pi) const { return data_at(pi.x(), pi.y()); }

  [[nodiscard]] auto cell_near(double x, double y) const {
    const auto xi = static_cast<int>(std::floor(x / resolution()));
    const auto yi = static_cast<int>(std::floor(y / resolution()));
    return Eigen::Vector2i{xi, yi};
  }

  [[nodiscard]] auto cell_near(const Eigen::Vector2d& p) const { return cell_near(p.x(), p.y()); }

  [[nodiscard]] bool free_at(std::size_t index) const {
    const auto data = data_at(index);
    if (!data.has_value()) {
      return false;
    }
    return !data.value();
  }

  [[nodiscard]] bool free_at(int xi, int yi) const { return free_at(index_at(xi, yi)); }

  [[nodiscard]] bool free_at(const Eigen::Vector2i& pi) const { return free_at(pi.x(), pi.y()); }

  [[nodiscard]] auto coordinates_at(int xi, int yi) const {
    return resolution() * Eigen::Vector2d{
                              (static_cast<double>(xi) + 0.5),
                              (static_cast<double>(yi) + 0.5),
                          };
  }

  [[nodiscard]] auto coordinates_at(const Eigen::Vector2i& pi) const { return coordinates_at(pi.x(), pi.y()); }

 private:
  double resolution_;
  static constexpr std::size_t kWidth = Cols;
  static constexpr std::size_t kHeight = Rows;

  std::array<bool, Rows * Cols> data_;
};

template <class Map>
std::optional<double> baseline_raycast(const Map& map, Eigen::Vector2i source, Eigen::Vector2i target) {
  const bool steep = std::abs(target.y() - source.y()) > std::abs(target.x() - source.x());

  if (steep) {
    std::swap(source.x(), source.y());
    std::swap(target.x(), target.y());
  }

  const auto delta = Eigen::Vector2i{
      std::abs(target.x() - source.x()),
      std::abs(target.y() - source.y()),
  };

  int error = 0;

  const auto step = Eigen::Vector2i{
      source.x() < target.x() ? 1 : -1,
      source.y() < target.y() ? 1 : -1,
  };

  auto current = source;

  do {
    if (steep) {
      if (map.data_at(current.y(), current.x()).value_or(true)) {
        break;
      }
    } else {
      if (map.data_at(current.x(), current.y()).value_or(true)) {
        break;
      }
    }

    if (current.x() == (target.x() + step.x())) {
      return std::nullopt;
    }

    current.x() += step.x();
    error += delta.y();
    if (delta.x() <= 2 * error) {
      current.y() += step.y();
      error -= delta.x();
    }
  } while (true);

  return (current - source).norm() * map.resolution();
}

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
    benchmark::DoNotOptimize(baseline_raycast(map, source, target));
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
