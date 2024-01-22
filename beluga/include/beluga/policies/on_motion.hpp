// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_POLICIES_ON_MOTION_HPP
#define BELUGA_POLICIES_ON_MOTION_HPP

#include <beluga/policies/policy.hpp>
#include <sophus/se2.hpp>

/**
 * \file
 * \brief Defines a policy for triggering an action based on motion in SE2 space.
 */

namespace beluga::policies {

namespace detail {

/// Implementation detail for the on_motion_policy.
/**
 * This policy triggers an action based on motion in SE2 space, where motion is determined
 * by a minimum distance and a minimum angle threshold.
 *
 * \tparam Scalar The scalar type for Sophus::SE2.
 */
template <class Scalar>
struct on_motion_policy {
 public:
  static_assert(std::is_arithmetic_v<Scalar>);

  /// Constructor.
  /**
   * \param min_distance The minimum translation distance to trigger the action.
   * \param min_angle The minimum rotation angle (in radians) to trigger the action.
   */
  constexpr on_motion_policy(Scalar min_distance, Scalar min_angle)
      : min_distance_(min_distance), min_angle_(min_angle) {}

  /// Call operator overload for SE2 elements.
  /**
   * \param pose The SE2 pose to check for motion.
   * \return True if the action should be triggered based on motion, false otherwise.
   *
   * Checks the motion based on the provided SE2 pose, and triggers the action if the
   * motion surpasses the specified distance and angle thresholds.
   */
  constexpr bool operator()(const Sophus::SE2<Scalar>& pose) {
    if (!latest_pose_) {
      latest_pose_ = pose;
      return true;
    }

    const auto delta = latest_pose_->inverse() * pose;
    const bool delta_is_above_threshold =                     //
        std::abs(delta.translation().x()) > min_distance_ ||  //
        std::abs(delta.translation().y()) > min_distance_ ||  //
        std::abs(delta.so2().log()) > min_angle_;

    if (delta_is_above_threshold) {
      latest_pose_ = pose;
    }

    return delta_is_above_threshold;
  }

 private:
  Scalar min_distance_{0.0};                        ///< The minimum translation distance to trigger the action.
  Scalar min_angle_{0.0};                           ///< The minimum rotation angle (in radians) to trigger the action.
  std::optional<Sophus::SE2<Scalar>> latest_pose_;  ///< The latest SE2 pose for motion comparison.
};

/// Implementation detail for an on_motion_fn object.
struct on_motion_fn {
  /// Overload that creates the policy closure.
  template <class Scalar>
  constexpr auto operator()(Scalar min_distance, Scalar min_angle) const {
    return beluga::make_policy(on_motion_policy<Scalar>{min_distance, min_angle});
  }
};

}  // namespace detail

/// Policy that triggers an action based on motion in SE2 space.
/**
 * This policy is designed to be used for scenarios where an action needs to be performed
 * based on the detected motion in the SE2 space, considering specified distance and angle thresholds.
 */
inline constexpr detail::on_motion_fn on_motion;

}  // namespace beluga::policies

#endif
