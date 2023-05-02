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

#include <range/v3/range/conversion.hpp>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace {

void BM_Bresenham2i(benchmark::State& state, beluga::bresenham2i::Variant variant) {
  const auto N = state.range(0);
  const auto algorithm = beluga::bresenham2i{variant};
  for (auto _ : state) {
    Eigen::Vector2i storage;
    for (const auto& cell : algorithm({0, 0}, {N, N})) {
      benchmark::DoNotOptimize(storage = cell);
    }
  }
  state.SetComplexityN(N);
}

BENCHMARK_CAPTURE(BM_Bresenham2i, Standard, beluga::bresenham2i::STANDARD)
    ->RangeMultiplier(2)
    ->Range(128, 4096)
    ->Complexity();

BENCHMARK_CAPTURE(BM_Bresenham2i, Modified, beluga::bresenham2i::MODIFIED)
    ->RangeMultiplier(2)
    ->Range(128, 4096)
    ->Complexity();

template <std::size_t Rows, std::size_t Cols>
class StaticOccupancyGrid {
 public:
  struct Traits {
    static bool is_free(bool value) { return !value; }
    static bool is_unknown(bool) { return false; }
    static bool is_occupied(bool value) { return value; }
  };

  explicit StaticOccupancyGrid(
      std::array<bool, Rows * Cols> array,
      double resolution = 1.0,
      const Sophus::SE2d& origin = Sophus::SE2d{})
      : grid_{array}, origin_{origin}, resolution_{resolution} {}

  [[nodiscard]] std::size_t size() const { return grid_.size(); }

  [[nodiscard]] auto& data() { return grid_; }

  [[nodiscard]] const auto& data() const { return grid_; }

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] Eigen::Vector2i cell(double x, double y) const {
    const auto xi = static_cast<std::size_t>(std::max(std::floor(x / resolution()), 0.));
    const auto yi = static_cast<std::size_t>(std::max(std::floor(y / resolution()), 0.));
    return Eigen::Vector2i{static_cast<int>(std::min(xi, width() - 1)), static_cast<int>(std::min(yi, height() - 1))};
  }

  [[nodiscard]] Eigen::Vector2i cell(const Eigen::Vector2d& point) const { return cell(point.x(), point.y()); }

  [[nodiscard]] std::size_t index(int xi, int yi) const {
    if (xi < 0 || static_cast<std::size_t>(xi) >= width()) {
      return size();
    }
    if (yi < 0 || static_cast<std::size_t>(yi) >= height()) {
      return size();
    }
    return xi + yi * width();
  }

  [[nodiscard]] std::size_t index(const Eigen::Vector2i& cell) const { return index(cell.x(), cell.y()); }

  [[nodiscard]] Eigen::Vector2d point(int xi, int yi) const {
    return Eigen::Vector2d{
        (static_cast<double>(xi) + 0.5) * resolution(), (static_cast<double>(yi) + 0.5) * resolution()};
  }

  [[nodiscard]] Eigen::Vector2d point(const Eigen::Vector2i& cell) const { return point(cell.x(), cell.y()); }

  [[nodiscard]] double resolution() const { return resolution_; }
  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }

 private:
  std::array<bool, Rows * Cols> grid_;
  Sophus::SE2d origin_;
  double resolution_;
};

void BM_RayCasting2d(benchmark::State& state) {
  constexpr double kMaxRange = 100.0;
  constexpr double kResolution = 0.05;

  const auto N = static_cast<int>(state.range(0));
  auto grid = StaticOccupancyGrid<1280, 1280>{{}, kResolution};
  grid.data()[grid.index(N, N)] = true;

  const auto source_pose = Sophus::SE2d{0., Eigen::Vector2d{1., 1.}};
  const auto beam_bearing = Sophus::SO2d{Sophus::Constants<double>::pi() / 4.};
  const auto beam = beluga::ray2d{grid, source_pose, kMaxRange};
  for (auto _ : state) {
    benchmark::DoNotOptimize(beam.cast(beam_bearing));
  }
  state.SetComplexityN(N);
}

BENCHMARK(BM_RayCasting2d)->RangeMultiplier(2)->Range(128, 1024)->Complexity();

}  // namespace
