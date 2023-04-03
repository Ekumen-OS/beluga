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

#include <gmock/gmock.h>
#include "beluga/algorithm/raycasting.hpp"
#include "beluga/sensor.hpp"
#include "beluga/sensor/beam_model.hpp"
#include "ciabatta/ciabatta.hpp"

namespace beluga {

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
      : grid_{array}, origin_{origin}, origin_inverse_{origin.inverse()}, resolution_{resolution} {}

  [[nodiscard]] std::size_t size() const { return grid_.size(); }

  [[nodiscard]] const auto& data() const { return grid_; }

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] const Sophus::SE2d& origin_inverse() const { return origin_inverse_; }

  [[nodiscard]] std::size_t index(double x, double y) const {
    const auto x_index = static_cast<std::size_t>(std::floor(x / resolution()));
    const auto y_index = static_cast<std::size_t>(std::floor(y / resolution()));
    return index(x_index, y_index);
  }

  [[nodiscard]] std::size_t index(std::size_t x_index, std::size_t y_index) const {
    if (x_index >= width() || y_index >= height()) {
      return size();  // If the point is outside the map, return an invalid index
    }
    return x_index + y_index * width();
  }

  [[nodiscard]] std::size_t index(const Eigen::Vector2d& point) const { return index(point.x(), point.y()); }

  [[nodiscard]] Eigen::Vector2d point(std::size_t index) const {
    return Eigen::Vector2d{
        (static_cast<double>(index % width()) + 0.5) * resolution(),
        (static_cast<double>(index / width()) + 0.5) * resolution()};  // NOLINT(bugprone-integer-division)
  }

  [[nodiscard]] auto neighbors(std::size_t index) const {
    auto result = std::vector<std::size_t>{};
    const std::size_t row = index / width();
    const std::size_t col = index % width();
    if (row < (height() - 1)) {
      result.push_back(index + width());
    }
    if (row > 0) {
      result.push_back(index - width());
    }
    if (col < (width() - 1)) {
      result.push_back(index + 1);
    }
    if (col > 0) {
      result.push_back(index - 1);
    }
    return result;
  }
  [[nodiscard]] double resolution() const { return resolution_; }

 private:
  std::array<bool, Rows * Cols> grid_;
  Sophus::SE2d origin_;
  Sophus::SE2d origin_inverse_;
  double resolution_;

  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }
};

TEST(Raycasting, casting) {
  constexpr double kResolution = 0.5;
  // Note that axes are:
  // Positive X -> Right
  // Positive Y -> Down

  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution};
  // clang-format on

  // Horizontal ray that hits the middle occupied cell.
  double ray_len = raycast(grid, Sophus::SE2d{0, Eigen::Vector2d{0.5, 0.}}, Sophus::SO2d{0}, 5);
  EXPECT_NEAR(ray_len, 2.5495097567963922, 1E-8);

  // Downwards ray that hits the map boundary.
  ray_len =
      raycast(grid, Sophus::SE2d{0, Eigen::Vector2d{0., 1.}}, Sophus::SO2d{Sophus::Constants<double>::pi() / 2.}, 5);
  EXPECT_NEAR(ray_len, 2.6925824035672519, 1E-8);

  // Start cell is occupied, should return 0.
  ray_len =
      raycast(grid, Sophus::SE2d{0, Eigen::Vector2d{1., 1.}}, Sophus::SO2d{Sophus::Constants<double>::pi() / 2.}, 5);
  EXPECT_EQ(ray_len, 0);

  // Downwards ray that is limited by beam range.
  ray_len =
      raycast(grid, Sophus::SE2d{0, Eigen::Vector2d{0., 0.}}, Sophus::SO2d{Sophus::Constants<double>::pi() / 2.}, 1);
  EXPECT_EQ(ray_len, 1);

  // Downwards ray that hits the occupied cell.
  ray_len =
      raycast(grid, Sophus::SE2d{0, Eigen::Vector2d{1., 0.}}, Sophus::SO2d{Sophus::Constants<double>::pi() / 2.}, 5);
  EXPECT_EQ(ray_len, 1);

  // Diagonal ray that hits the occupied cell.
  ray_len =
      raycast(grid, Sophus::SE2d{0, Eigen::Vector2d{0., 0.}}, Sophus::SO2d{Sophus::Constants<double>::pi() / 4.}, 5);
  EXPECT_EQ(ray_len, std::sqrt(2));
}
}  // namespace beluga
