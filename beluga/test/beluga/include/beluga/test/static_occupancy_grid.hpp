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
#include <initializer_list>
#include <vector>

#include <beluga/sensor/data/dense_grid.hpp>
#include <beluga/sensor/data/plain_grid_storage.hpp>
#include <beluga/sensor/data/regular_grid.hpp>

#include <ciabatta/ciabatta.hpp>

#include "beluga/sensor/data/occupancy_grid.hpp"

#include <sophus/se2.hpp>

namespace beluga::testing {

template <class Mixin>
class StaticOccupancyGridMixin : public Mixin {
 public:
  using PlainGridStorage = beluga::PlainGridStorage<bool>;

  struct ValueTraits {
    [[nodiscard]] bool is_free(bool value) const { return !value; }
    [[nodiscard]] bool is_unknown(bool) const { return false; }
    [[nodiscard]] bool is_occupied(bool value) const { return value; }
  };

  template <typename... Args>
  explicit StaticOccupancyGridMixin(
      PlainGridStorage&& grid_storage,
      double resolution,
      const Sophus::SE2d& origin,
      Args&&... args)
      : Mixin(std::forward<Args>(args)...),
        grid_storage_{std::move(grid_storage)},
        origin_(origin),
        resolution_{resolution} {}

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] std::size_t size() const { return grid_storage_.size(); }

  [[nodiscard]] std::size_t width() const { return grid_storage_.width(); }

  [[nodiscard]] std::size_t height() const { return grid_storage_.height(); }

  [[nodiscard]] double resolution() const { return resolution_; }

  [[nodiscard]] auto value_traits() const { return ValueTraits{}; }

  [[nodiscard]] const auto& cell(const int x, const int y) const { return grid_storage_.cell(x, y); }
  [[nodiscard]] auto& cell(const int x, const int y) { return grid_storage_.cell(x, y); }

 private:
  PlainGridStorage grid_storage_;
  Sophus::SE2d origin_;
  double resolution_;
};

using StaticOccupancyGrid = ciabatta::mixin<
    ciabatta::curry<StaticOccupancyGridMixin>::template mixin,
    BaseOccupancyGrid2Mixin,
    BaseDenseGrid2Mixin,
    BaseRegularGrid2Mixin>;

}  // namespace beluga::testing

#endif
