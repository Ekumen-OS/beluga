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

#include "beluga_amcl/particle_filtering.hpp"

#include <random>

#include <beluga/random/multivariate_normal_distribution.hpp>

#include <range/v3/view/generate.hpp>

#include <sophus/so2.hpp>
#include <sophus/se2.hpp>

namespace beluga_amcl
{

void initialize_with_pose(
  const Sophus::SE2d & pose,
  const Eigen::Matrix3d & covariance,
  LaserLocalizationInterface2d * particle_filter)
{
  const auto mean =
    Eigen::Vector3d{pose.translation().x(), pose.translation().y(), pose.so2().log()};
  auto distribution = beluga::MultivariateNormalDistribution{mean, covariance};
  particle_filter->initialize_states(
    ranges::views::generate(
      [&distribution]() mutable {
        static auto generator = std::mt19937{std::random_device()()};
        const auto sample = distribution(generator);
        return Sophus::SE2d{Sophus::SO2d{sample.z()}, Eigen::Vector2d{sample.x(), sample.y()}};
      }));
}

}  // namespace beluga_amcl
