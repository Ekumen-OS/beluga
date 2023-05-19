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

template <std::size_t Rows, std::size_t Cols>
class StaticOccupancyGrid : public BaseOccupancyGrid2<StaticOccupancyGrid<Rows, Cols>> {
 public:
  struct ValueTraits {
    [[nodiscard]] bool is_free(bool value) const { return !value; }
    [[nodiscard]] bool is_unknown(bool) const { return false; }
    [[nodiscard]] bool is_occupied(bool value) const { return value; }
  };

  explicit StaticOccupancyGrid(
      std::array<bool, Rows * Cols> array,
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

  [[nodiscard]] auto value_traits() const { return ValueTraits{}; }

 private:
  std::array<bool, Rows * Cols> grid_;
  Sophus::SE2d origin_;
  double resolution_;
};

}  // namespace beluga::testing

#endif
