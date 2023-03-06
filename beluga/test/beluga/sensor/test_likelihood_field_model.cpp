// Copyright 2022 Ekumen, Inc.
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

#include <beluga/sensor.hpp>
#include <ciabatta/ciabatta.hpp>

namespace {

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

  std::size_t size() const { return grid_.size(); }
  const auto& data() const { return grid_; }
  const Sophus::SE2d& origin() const { return origin_; }
  const Sophus::SE2d& origin_inverse() const { return origin_inverse_; }

  std::size_t index(double x, double y) const {
    const auto x_index = static_cast<std::size_t>(std::floor(x / resolution()));
    const auto y_index = static_cast<std::size_t>(std::floor(y / resolution()));
    if (x_index >= width() || y_index >= height()) {
      return size();  // If the point is outside the map, return an invalid index
    }
    return x_index + y_index * width();
  }

  std::size_t index(const Eigen::Vector2d& point) const { return index(point.x(), point.y()); }

  Eigen::Vector2d point(std::size_t index) const {
    return Eigen::Vector2d{
        (static_cast<double>(index % width()) + 0.5) * resolution(),
        (static_cast<double>(index / width()) + 0.5) * resolution()};
  }

  auto neighbors(std::size_t index) const {
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

 private:
  std::array<bool, Rows * Cols> grid_;
  Sophus::SE2d origin_;
  Sophus::SE2d origin_inverse_;
  double resolution_;

  std::size_t width() const { return Cols; }
  std::size_t height() const { return Rows; }
  double resolution() const { return resolution_; }
};

using UUT = ciabatta::mixin<
    ciabatta::curry<beluga::LikelihoodFieldModel, StaticOccupancyGrid<5, 5>>::mixin,
    ciabatta::provides<beluga::LaserSensorModelInterface2d>::mixin>;

TEST(LikelihoodFieldModel, LikelihoodField) {
  constexpr double kResolution = 0.5;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, true ,
    false, false, false, true , false,
    false, false, true , false, false,
    false, true , false, false, false,
    true , false, false, false, false},
    kResolution};

  const double expected_likelihood_field[] = {
    0.025, 0.025, 0.025, 0.070, 1.020,
    0.025, 0.027, 0.070, 1.020, 0.070,
    0.025, 0.070, 1.020, 0.070, 0.025,
    0.070, 1.020, 0.070, 0.027, 0.025,
    1.020, 0.070, 0.025, 0.025, 0.025
  };
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto mixin = UUT{params, grid};
  ASSERT_THAT(mixin.likelihood_field(), testing::Pointwise(testing::DoubleNear(0.003), expected_likelihood_field));
}

TEST(LikelihoodFieldModel, ImportanceWeight) {
  constexpr double kResolution = 0.5;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto mixin = UUT{params, grid};

  mixin.update_sensor(std::vector<std::pair<double, double>>{{1.25, 1.25}});
  ASSERT_NEAR(1.020, mixin.importance_weight(grid.origin()), 0.003);

  mixin.update_sensor(std::vector<std::pair<double, double>>{{2.25, 2.25}});
  ASSERT_NEAR(0.025, mixin.importance_weight(grid.origin()), 0.003);

  mixin.update_sensor(std::vector<std::pair<double, double>>{{-50.0, 50.0}});
  ASSERT_NEAR(0.000, mixin.importance_weight(grid.origin()), 0.003);

  mixin.update_sensor(std::vector<std::pair<double, double>>{{1.20, 1.20}, {1.25, 1.25}, {1.30, 1.30}});
  ASSERT_NEAR(3.060, mixin.importance_weight(grid.origin()), 0.01);

  mixin.update_sensor(std::vector<std::pair<double, double>>{{0.0, 0.0}});
  ASSERT_NEAR(1.020, mixin.importance_weight(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{1.25, 1.25}}), 0.003);
}

TEST(LikelihoodFieldModel, GridWithOffset) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, true },
    kResolution,
    Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{-5, -5}}};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto mixin = UUT{params, grid};

  mixin.update_sensor(std::vector<std::pair<double, double>>{{4.5, 4.5}});
  ASSERT_NEAR(1.020, mixin.importance_weight(Sophus::SE2d{}), 0.003);

  mixin.update_sensor(std::vector<std::pair<double, double>>{{9.5, 9.5}});
  ASSERT_NEAR(1.020, mixin.importance_weight(grid.origin()), 0.003);
}

TEST(LikelihoodFieldModel, GridWithRotation) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, true },
    kResolution,
    Sophus::SE2d{Sophus::SO2d{Sophus::Constants<double>::pi() / 2}, Eigen::Vector2d{0.0, 0.0}}};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto mixin = UUT{params, grid};

  mixin.update_sensor(std::vector<std::pair<double, double>>{{-9.5, 9.5}});
  ASSERT_NEAR(1.020, mixin.importance_weight(Sophus::SE2d{}), 0.003);

  mixin.update_sensor(std::vector<std::pair<double, double>>{{9.5, 9.5}});
  ASSERT_NEAR(1.020, mixin.importance_weight(grid.origin()), 0.003);
}

TEST(LikelihoodFieldModel, GridWithRotationAndOffset) {
  constexpr double kResolution = 2.0;
  // clang-format off
  const auto origin_rotation = Sophus::SO2d{Sophus::Constants<double>::pi() / 2};
  const auto origin = Sophus::SE2d{origin_rotation, origin_rotation * Eigen::Vector2d{-5, -5}};

  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, false, false, true },
    kResolution,
    origin};
  // clang-format on

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto mixin = UUT{params, grid};

  mixin.update_sensor(std::vector<std::pair<double, double>>{{-4.5, 4.5}});
  ASSERT_NEAR(1.020, mixin.importance_weight(Sophus::SE2d{}), 0.003);

  mixin.update_sensor(std::vector<std::pair<double, double>>{{9.5, 9.5}});
  ASSERT_NEAR(1.020, mixin.importance_weight(grid.origin()), 0.003);
}

}  // namespace
