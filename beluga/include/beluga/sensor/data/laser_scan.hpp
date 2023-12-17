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

#ifndef BELUGA_SENSOR_DATA_LASER_SCAN_HPP
#define BELUGA_SENSOR_DATA_LASER_SCAN_HPP

#include <cmath>

#include <ciabatta/ciabatta.hpp>

#include <sophus/types.hpp>

#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

/**
 * \file
 * \brief Implementation of a laser scan interface.
 */

namespace beluga {

/**
 * \page LaserScanPage Beluga named requirements: LaserScan
 *
 * A type `L` satisfies `LaserScan` requirements if given `l` a possibly
 * const instance of `L`:
 * - `L::Scalar` is defined and is a scalar type to be used for range and angle values,
 * - `l.ranges()` returns a view to the beam ranges,
 * - `l.angles()` returns a view to the corresponding beam angles in sensor frame,
 * - `l.min_range()` returns the minimum allowed range value,
 * - `l.max_range()` returns the maximum allowed range value,
 * - `l.origin()` returns the transform, of `Sophus::SE3d` type, from the global to local
 *   frame in the sensor space;
 */

/// Laser scan 2D base type.
/**
 * When instantiated, it satisfies \ref LaserScanPage.
 *
 * \tparam Derived Concrete laser scan type. It must define
 * `Derived::ranges()`, `Derived::angles()`,
 * `Derived::min_range()`, `Derived::max_range()`,
 * as described in \ref LaserScanPage.
 */
template <typename Derived>
class BaseLaserScan : public ciabatta::ciabatta_top<Derived> {
 public:
  /// View points in cartesian coordinates in sensor frame.
  [[nodiscard]] auto points_in_cartesian_coordinates() const {
    return this->self().points_in_polar_coordinates() | ranges::views::transform([](const auto& vector) {
             using std::cos, std::sin;
             return Sophus::Vector2<typename Derived::Scalar>{
                 vector.x() * cos(vector.y()), vector.x() * sin(vector.y())};
           });
  }

  /// View points in polar coordinates in sensor frame.
  /**
   * Range values lower than min_range or greater than max_range will be discarded.
   * Same for NaN values.
   */
  [[nodiscard]] auto points_in_polar_coordinates() const {
    static_assert(std::is_floating_point_v<ranges::range_value_t<decltype(this->self().ranges())>>);
    static_assert(std::is_floating_point_v<ranges::range_value_t<decltype(this->self().angles())>>);
    return ranges::views::zip(this->self().ranges(), this->self().angles()) |
           ranges::views::filter([this](const auto& tuple) {
             const auto [range, theta] = tuple;
             using std::isnan;
             return !isnan(range) && range >= this->self().min_range() && range <= this->self().max_range();
           }) |
           ranges::views::transform([](const auto& tuple) {
             const auto [range, theta] = tuple;
             return Sophus::Vector2<typename Derived::Scalar>{range, theta};
           });
  }
};

}  // namespace beluga

#endif
