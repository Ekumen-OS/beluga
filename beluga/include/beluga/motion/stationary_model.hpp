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

#ifndef BELUGA_MOTION_STATIONARY_MODEL_HPP
#define BELUGA_MOTION_STATIONARY_MODEL_HPP

#include <random>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace beluga {

template <class Mixin>
class StationaryModel : public Mixin {
 public:
  template <class... Args>
  explicit StationaryModel(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] Sophus::SE2d apply_motion(const Sophus::SE2d& state) const {
    static thread_local std::mt19937 generator{std::random_device()()};
    auto distribution = std::normal_distribution<>{0, 0.02};
    return state * Sophus::SE2d{
                       Sophus::SO2d{distribution(generator)},
                       Eigen::Vector2d{distribution(generator), distribution(generator)}};
  }
};

}  // namespace beluga

#endif
