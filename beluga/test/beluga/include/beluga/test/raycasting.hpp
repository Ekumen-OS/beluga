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

#ifndef BELUGA_TEST_RAYCASTING_HPP
#define BELUGA_TEST_RAYCASTING_HPP

#include <Eigen/Core>

namespace beluga::testing {

template <class Map>
std::optional<double> raycast(const Map& map, Eigen::Vector2i source, Eigen::Vector2i target) {
  const bool steep = std::abs(target.y() - source.y()) > std::abs(target.x() - source.x());

  if (steep) {
    std::swap(source.x(), source.y());
    std::swap(target.x(), target.y());
  }

  const auto delta = Eigen::Vector2i{
      std::abs(target.x() - source.x()),
      std::abs(target.y() - source.y()),
  };

  int error = 0;

  const auto step = Eigen::Vector2i{
      source.x() < target.x() ? 1 : -1,
      source.y() < target.y() ? 1 : -1,
  };

  auto current = source;

  do {
    if (steep) {
      if (auto opt = map.data_at(current.y(), current.x()); opt.has_value()) {
        if (opt.value()) {
          break;
        }
      } else {
        return std::nullopt;
      }
    } else {
      if (auto opt = map.data_at(current.x(), current.y()); opt.has_value()) {
        if (opt.value()) {
          break;
        }
      } else {
        return std::nullopt;
      }
    }

    current.x() += step.x();
    error += delta.y();
    if (delta.x() <= 2 * error) {
      current.y() += step.y();
      error -= delta.x();
    }

    if (current.x() == (target.x() + step.x())) {
      return std::nullopt;
    }
  } while (true);

  return (current - source).cast<double>().norm() * map.resolution();
}

}  // namespace beluga::testing

#endif
