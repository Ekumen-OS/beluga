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

#include "beluga/algorithm/raycasting.hpp"

#include "beluga/sensor.hpp"
#include "beluga/sensor/beam_model.hpp"
#include "ciabatta/ciabatta.hpp"

#include <gmock/gmock.h>

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
  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }

 private:
  std::array<bool, Rows * Cols> grid_;
  Sophus::SE2d origin_;
  Sophus::SE2d origin_inverse_;
  double resolution_;
};

TEST(Raycasting, StandardBresenham) {
  auto algorithm = Bresenham2i{Bresenham2i::kStandard};

  {
    // +---+---+
    // |   | > |
    // +---+---+
    // | > |   |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}, {1, 1}};
    const auto trace = algorithm({0, 0}, {1, 1}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // |   | < |
    // +---+---+
    // | < |   |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{1, 1}, {0, 0}};
    const auto trace = algorithm({1, 1}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+
    // |   |   | > |
    // +---+---+---+
    // | > | > |   |
    // +---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}, {1, 0}, {2, 1}};
    const auto trace = algorithm({0, 0}, {2, 1}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+
    // |   | < | < |
    // +---+---+---+
    // | < |   |   |
    // +---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{2, 1}, {1, 1}, {0, 0}};
    const auto trace = algorithm({2, 1}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // | v |   |
    // +---+---+
    // | v |   |
    // +---+---+
    // | v |   |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 2}, {0, 1}, {0, 0}};
    const auto trace = algorithm({0, 2}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+---+
    // |   |   |   | < |
    // +---+---+---+---+
    // |   | < | < |   |
    // +---+---+---+---+
    // | < |   |   |   |
    // +---+---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{3, 2}, {2, 1}, {1, 1}, {0, 0}};
    const auto trace = algorithm({3, 2}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }
}

TEST(Raycasting, ModifiedBresenham) {
  auto algorithm = Bresenham2i{Bresenham2i::kModified};

  {
    // +---+---+
    // | > | > |
    // +---+---+
    // | > | > |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}, {1, 0}, {0, 1}, {1, 1}};
    const auto trace = algorithm({0, 0}, {1, 1}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // | < | < |
    // +---+---+
    // | < | < |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{1, 1}, {0, 1}, {1, 0}, {0, 0}};
    const auto trace = algorithm({1, 1}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+
    // |   | > | > |
    // +---+---+---+
    // | > | > |   |
    // +---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}, {1, 0}, {1, 1}, {2, 1}};
    const auto trace = algorithm({0, 0}, {2, 1}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+
    // |   | < | < |
    // +---+---+---+
    // | < | < |   |
    // +---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{2, 1}, {1, 1}, {1, 0}, {0, 0}};
    const auto trace = algorithm({2, 1}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // | v |   |
    // +---+---+
    // | v |   |
    // +---+---+
    // | v |   |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 2}, {0, 1}, {0, 0}};
    const auto trace = algorithm({0, 2}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+---+
    // |   |   | < | < |
    // +---+---+---+---+
    // |   | < | < |   |
    // +---+---+---+---+
    // | < | < |   |   |
    // +---+---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{3, 2}, {2, 2}, {2, 1}, {1, 1}, {1, 0}, {0, 0}};
    const auto trace = algorithm({3, 2}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }
}

TEST(Raycasting, Nominal) {
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

  constexpr double kMaxRange = 5.;

  {
    // Horizontal ray that hits the map boundary.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0.5, 0.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    EXPECT_EQ(ray.cast(Sophus::SO2d{0.}), std::nullopt);
  }

  {
    // Horizontal ray that hits the occupied cell.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0., 1.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    EXPECT_EQ(ray.cast(Sophus::SO2d{0.}), 1.);
  }

  {
    // Downwards ray that hits the map boundary.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0., 1.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{Sophus::Constants<double>::pi() / 2.});
    EXPECT_EQ(distance, std::nullopt);
  }

  {
    // Start cell is occupied, should return 0.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{1., 1.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{Sophus::Constants<double>::pi() / 2.});
    EXPECT_EQ(distance, 0.);
  }

  {
    // Downwards ray that is limited by beam range.
    constexpr double kShortenedRange = 1.;
    const auto pose = Sophus::SE2d{Sophus::Constants<double>::pi() / 2., Eigen::Vector2d{0., 0.}};
    const auto ray = Ray2d{grid, pose, kShortenedRange};
    EXPECT_EQ(ray.cast(Sophus::SO2d{0}), std::nullopt);
  }

  {
    // Downwards ray that hits the occupied cell.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{1., 0.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{Sophus::Constants<double>::pi() / 2.});
    EXPECT_EQ(distance, 1.);
  }

  {
    // Diagonal ray that hits the occupied cell.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0., 0.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{Sophus::Constants<double>::pi() / 4.});
    EXPECT_EQ(distance, std::sqrt(2));
  }
}

TEST(Raycasting, NonIdentityGridOrigin) {
  constexpr double kResolution = 0.5;

  const auto origin = Sophus::SE2d{Sophus::SO2d{-Sophus::Constants<double>::pi() / 4.}, Eigen::Vector2d{0.5, 0.}};
  // Note that axes are:
  // Positive X -> Diagonal downwards right
  // Positive Y -> Diagonal downwards left

  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution, origin};
  // clang-format on

  constexpr double kMaxRange = 5.;

  {
    // Diagonal ray that hits the occupied cell.
    const auto pose = Sophus::SE2d{0., Eigen::Vector2d{0.5, 0.}};
    const auto ray = Ray2d{grid, pose, kMaxRange};
    const auto distance = ray.cast(Sophus::SO2d{0.});
    EXPECT_EQ(distance, std::sqrt(2));
  }
}

}  // namespace beluga
