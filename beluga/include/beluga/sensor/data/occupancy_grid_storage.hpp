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

#ifndef BELUGA_STATIC_OCCUPANCY_GRID_HPP
#define BELUGA_STATIC_OCCUPANCY_GRID_HPP

#include <cstdint>

#include <ciabatta/ciabatta.hpp>
#include <sophus/se2.hpp>

#include "beluga/sensor/data/linear_grid_storage.hpp"

namespace beluga {

template <class Mixin, typename ValueType, typename ValueTraitsType>
class OccupancyStorageMixin : public Mixin {
 public:
  using MapStorage = LinearGridStorage<ValueType>;
  using ValueTraits = ValueTraitsType;

  template <typename... Args>
  explicit OccupancyStorageMixin(
      MapStorage&& grid_storage,
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
  MapStorage grid_storage_;
  Sophus::SE2d origin_;
  double resolution_;
};

}  // namespace beluga

#endif
