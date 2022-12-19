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

#pragma once

#include <optional>
#include <random>
#include <shared_mutex>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace beluga {

struct DifferentialDriveModelParam {
  double rotation_noise_from_rotation;
  double rotation_noise_from_translation;
  double translation_noise_from_translation;
  double translation_noise_from_rotation;
  double distance_threshold = 0.01; /* Distance threshold to detect in-place rotation */
};

template <class Mixin>
class DifferentialDriveModel : public Mixin {
 public:
  using DistributionParam = typename std::normal_distribution<double>::param_type;

  template <class... Args>
  explicit DifferentialDriveModel(const DifferentialDriveModelParam& params, Args&&... args)
      : Mixin(std::forward<Args>(args)...), params_{params} {}

  [[nodiscard]] Sophus::SE2d apply_motion(const Sophus::SE2d& state) const {
    static thread_local auto generator = std::mt19937{std::random_device()()};
    static thread_local auto distribution = std::normal_distribution<double>{};

    const auto lock = std::shared_lock<std::shared_mutex>{params_mutex_};
    const auto first_rotation = Sophus::SO2d{distribution(generator, first_rotation_params_)};
    const auto translation = Eigen::Vector2d{distribution(generator, translation_params_), 0.0};
    const auto second_rotation = Sophus::SO2d{distribution(generator, second_rotation_params_)};
    return state * Sophus::SE2d{first_rotation, Eigen::Vector2d{0.0, 0.0}} * Sophus::SE2d{second_rotation, translation};
  }

  void update_motion(const Sophus::SE2d& pose) {
    if (last_pose_) {
      const auto translation = pose.translation() - last_pose_.value().translation();
      const double distance = translation.norm();
      const double distance_variance = distance * distance;

      const auto& previous_orientation = last_pose_.value().so2();
      const auto& current_orientation = pose.so2();
      const auto first_rotation =
          distance > params_.distance_threshold
              ? Sophus::SO2d{std::atan2(translation.y(), translation.x())} * previous_orientation.inverse()
              : Sophus::SO2d{0.0};
      const auto second_rotation = current_orientation * previous_orientation.inverse() * first_rotation.inverse();
      const auto combined_rotation = first_rotation * second_rotation;

      {
        const auto lock = std::lock_guard<std::shared_mutex>{params_mutex_};
        first_rotation_params_ = DistributionParam{
            first_rotation.log(), std::sqrt(
                                      params_.rotation_noise_from_rotation * rotation_variance(first_rotation) +
                                      params_.rotation_noise_from_translation * distance_variance)};
        translation_params_ = DistributionParam{
            distance, std::sqrt(
                          params_.translation_noise_from_translation * distance_variance +
                          params_.translation_noise_from_rotation * rotation_variance(combined_rotation))};
        second_rotation_params_ = DistributionParam{
            second_rotation.log(), std::sqrt(
                                       params_.rotation_noise_from_rotation * rotation_variance(second_rotation) +
                                       params_.rotation_noise_from_translation * distance_variance)};
      }
    }
    last_pose_ = pose;
  }

 private:
  DifferentialDriveModelParam params_;
  std::optional<Sophus::SE2d> last_pose_;

  DistributionParam first_rotation_params_{0.0, 0.0};
  DistributionParam second_rotation_params_{0.0, 0.0};
  DistributionParam translation_params_{0.0, 0.0};
  mutable std::shared_mutex params_mutex_;

  static double rotation_variance(const Sophus::SO2d& rotation) {
    // Treat backward and forward motion symmetrically for the noise models.
    const auto flipped_rotation = rotation * Sophus::SO2d{Sophus::Constants<double>::pi()};
    const auto delta = std::min(std::abs(rotation.log()), std::abs(flipped_rotation.log()));
    return delta * delta;
  }
};

}  // namespace beluga
