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

#include <ciabatta/ciabatta.hpp>
#include <sophus/se2.hpp>

#include <beluga/sensor/data/dense_grid.hpp>
#include <beluga/sensor/data/linear_grid_storage.hpp>
#include <beluga/sensor/data/occupancy_grid.hpp>
#include <beluga/sensor/data/occupancy_grid_storage.hpp>
#include <beluga/sensor/data/regular_grid.hpp>

namespace beluga::testing {

using TestMapValueType = bool;

struct TestMapValueTraits {
  [[nodiscard]] bool is_free(bool value) const { return !value; }
  [[nodiscard]] bool is_unknown(bool) const { return false; }
  [[nodiscard]] bool is_occupied(bool value) const { return value; }
};

using StaticOccupancyGrid = ciabatta::mixin<
    ciabatta::curry<OccupancyStorageMixin, TestMapValueType, TestMapValueTraits, LinearGridStorage<TestMapValueType>>::
        template mixin,
    BaseOccupancyGrid2Mixin,
    BaseDenseGrid2Mixin,
    BaseRegularGrid2Mixin>;

}  // namespace beluga::testing

#endif
