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

#ifndef BELUGA_TEST_STATIC_OCCUPANCY_GRID_HPP
#define BELUGA_TEST_STATIC_OCCUPANCY_GRID_HPP

#include <array>
#include <cstdint>

#include "beluga/sensor/data/occupancy_grid.hpp"

#include <sophus/se2.hpp>

namespace beluga::testing {

template <class T>
struct ValueTraits;

template <>
struct ValueTraits<bool> {
  [[nodiscard]] static bool is_free(bool value) { return !value; }
  [[nodiscard]] static bool is_unknown(bool) { return false; }
  [[nodiscard]] static bool is_occupied(bool value) { return value; }
};

template <>
struct ValueTraits<std::int8_t> {
  static constexpr std::int8_t kFreeValue = 0;
  static constexpr std::int8_t kUnknownValue = -1;
  static constexpr std::int8_t kOccupiedValue = 100;

  [[nodiscard]] static bool is_free(std::int8_t value) { return value == kFreeValue; }
  [[nodiscard]] static bool is_unknown(std::int8_t value) { return value == kUnknownValue; }
  [[nodiscard]] static bool is_occupied(std::int8_t value) { return value == kOccupiedValue; }
};

template <std::size_t Rows, std::size_t Cols, class T = bool>
class StaticOccupancyGrid : public BaseOccupancyGrid2<StaticOccupancyGrid<Rows, Cols, T>> {
 public:
  explicit StaticOccupancyGrid(
      std::array<T, Rows * Cols> array,
      double resolution = 1.0,
      const Sophus::SE2d& origin = Sophus::SE2d{})
      : grid_{array}, origin_(origin), resolution_{resolution} {}

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] auto& data() { return grid_; }
  [[nodiscard]] const auto& data() const { return grid_; }
  [[nodiscard]] std::size_t size() const { return grid_.size(); }

  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }
  [[nodiscard]] double resolution() const { return resolution_; }

  [[nodiscard]] auto value_traits() const { return ValueTraits<T>{}; }

 private:
  std::array<T, Rows * Cols> grid_;
  Sophus::SE2d origin_;
  double resolution_;
};

}  // namespace beluga::testing

#endif
