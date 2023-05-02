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

#ifndef BELUGA_SENSOR_SCAN_HPP
#define BELUGA_SENSOR_SCAN_HPP

#include <cmath>

#include <Eigen/Core>

#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

namespace beluga {

///
template <typename Derived>
class RotatingBeamScan {
 public:
  [[nodiscard]] Derived* derived() { return static_cast<Derived*>(this); }

  [[nodiscard]] const Derived& derived() const { return static_cast<const Derived&>(*this); }

  [[nodiscard]] auto points_in_cartesian_coordinates() const {
    const auto& self = derived();
    return self.points_in_polar_coordinates() | ranges::view::transform([this](const auto& vector) {
             using std::cos, std::sin;
             return Eigen::Vector2<typename Derived::Scalar>{
                 vector.x() * cos(vector.y()), vector.x() * sin(vector.y())};
           });
  }

  [[nodiscard]] auto points_in_polar_coordinates() const {
    const auto& self = derived();
    return ranges::views::zip(self.ranges(), self.angles()) | ranges::view::filter([](const auto& tuple) {
             using std::isnan;
             const auto [r, theta] = tuple;
             return !(isnan(r) || isnan(theta));
           }) |
           ranges::view::transform([](const auto& tuple) {
             const auto [r, theta] = tuple;
             return Eigen::Vector2<typename Derived::Scalar>{r, theta};
           });
  }
};

}  // namespace beluga

#endif
