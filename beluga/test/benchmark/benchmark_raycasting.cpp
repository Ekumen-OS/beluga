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

  [[nodiscard]] bool valid(int xi, int yi) const {
    return xi >= 0 && xi < static_cast<int>(width()) && yi >= 0 && yi < static_cast<int>(height());
  }

  [[nodiscard]] bool valid(const Eigen::Vector2i& cell) const { return valid(cell.x(), cell.y()); }

  [[nodiscard]] std::size_t index(double x, double y) const {
    const auto xi = static_cast<int>(std::floor(x / resolution() + 0.5));
    const auto yi = static_cast<int>(std::floor(y / resolution() + 0.5));
    return index(xi, yi);
  }

  [[nodiscard]] std::size_t index(const Eigen::Vector2d& point) const { return index(point.x(), point.y()); }

  [[nodiscard]] std::size_t index(int xi, int yi) const {
    if (!valid(xi, yi)) {
      return size();
    }
    return static_cast<std::size_t>(xi) + static_cast<std::size_t>(yi) * width();
  }

  [[nodiscard]] std::size_t index(const Eigen::Vector2i& cell) const { return index(cell.x(), cell.y()); }

  [[nodiscard]] Eigen::Vector2i cell(double x, double y) const {
    const auto xi = static_cast<int>(std::floor(x / resolution() + 0.5));
    const auto yi = static_cast<int>(std::floor(y / resolution() + 0.5));
    return Eigen::Vector2i{xi, yi};
  }

  [[nodiscard]] Eigen::Vector2i cell(const Eigen::Vector2d& point) const { return cell(point.x(), point.y()); }

  [[nodiscard]] Eigen::Vector2d point(int xi, int yi) const {
    return Eigen::Vector2d{
        (static_cast<double>(xi) + 0.5) * resolution(), (static_cast<double>(yi) + 0.5) * resolution()};
  }

  [[nodiscard]] Eigen::Vector2d point(const Eigen::Vector2i& cell) const { return point(cell.x(), cell.y()); }

  [[nodiscard]] Eigen::Vector2d point(std::size_t index) const {
    return Eigen::Vector2d{
        (static_cast<double>(index % width()) + 0.5) * resolution(),
        (static_cast<double>(index / width()) + 0.5) * resolution()};  // NOLINT(bugprone-integer-division)
  }

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

  const auto n = static_cast<int>(state.range(0));
  auto grid = StaticOccupancyGrid<1280, 1280>{{}, kResolution};
  grid.data()[grid.index(n, n)] = true;

  const auto source_pose = Sophus::SE2d{0., Eigen::Vector2d{1., 1.}};
  const auto beam_bearing = Sophus::SO2d{Sophus::Constants<double>::pi() / 4.};
  const auto beam = beluga::Ray2d{grid, source_pose, kMaxRange};
  for (auto _ : state) {
    benchmark::DoNotOptimize(beam.cast(beam_bearing));
  }
  state.SetComplexityN(n);
}

BENCHMARK(BM_RayCasting2d)->RangeMultiplier(2)->Range(128, 1024)->Complexity();

}  // namespace
