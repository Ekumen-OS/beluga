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
 * \brief Defines a policy for triggering an action based on motion.
 */

namespace beluga::policies {

namespace detail {

/// Primary template for the on_motion_policy_base class.
/**
 * \tparam Pose The pose type to check for motion.
 */
template <class Pose>
struct on_motion_policy_base;

/// Specialization for on_motion_policy_base in SE2 space.
/**
 * \tparam Scalar The scalar type for SE2 elements.
 */
template <class Scalar>
struct on_motion_policy_base<Sophus::SE2<Scalar>> {
 public:
  /// Constructor.
  /**
   * \param min_distance The minimum translation distance to trigger the action.
   * \param min_angle The minimum rotation angle (in radians) to trigger the action.
   */
  constexpr on_motion_policy_base(Scalar min_distance, Scalar min_angle)
      : min_distance_(min_distance), min_angle_(min_angle) {}

  /// Return true if motion has been detected.
  /**
   * \param prev The previous SE2 pose to check for motion.
   * \param current The current SE2 pose to check for motion.
   * \return True if the action should be triggered based on motion, false otherwise.
   *
   * Checks the motion based on the provided SE2 pose, and triggers the action if the
   * motion surpasses the specified distance and angle thresholds.
   */
  constexpr bool operator()(const Sophus::SE2<Scalar>& prev, const Sophus::SE2<Scalar>& current) {
    const auto delta = prev.inverse() * current;
    return std::abs(delta.translation().x()) > min_distance_ ||  //
           std::abs(delta.translation().y()) > min_distance_ ||  //
           std::abs(delta.so2().log()) > min_angle_;
  }

 private:
  Scalar min_distance_{0.0};  ///< The minimum translation distance to trigger the action.
  Scalar min_angle_{0.0};     ///< The minimum rotation angle (in radians) to trigger the action.
};

/// Base implementation for the on_motion_policy algorithm.
/**
 * \tparam Pose The pose type to check for motion.
 */
template <class Pose>
struct on_motion_policy : public on_motion_policy_base<Pose> {
 public:
  using on_motion_policy_base<Pose>::on_motion_policy_base;
  using on_motion_policy_base<Pose>::operator();

  /// Return true if motion has been detected.
  constexpr bool operator()(const Pose& pose) {
    if (!latest_pose_) {
      latest_pose_ = pose;
      return true;
    }

    const bool moved = (*this)(*latest_pose_, pose);
    if (moved) {
      latest_pose_ = pose;
    }

    return moved;
  }

 private:
  std::optional<Pose> latest_pose_;  ///< The latest pose for motion comparison.
};

/// Implementation detail for the on_motion_fn object.
template <class Pose>
struct on_motion_fn {
  /// Overload that creates the policy closure.
  /**
   * This policy triggers an action based on motion.
   */
  template <class... Args>
  constexpr auto operator()(Args&&... args) const {
    return beluga::make_policy(on_motion_policy<Pose>{std::forward<Args>(args)...});
  }
};

}  // namespace detail

/// Policy that triggers an action based on motion.
/**
 * This policy is designed to be used for scenarios where an action needs to be performed
 * based on the detected motion, considering specified distance and angle thresholds.
 */
template <class Pose>
inline constexpr detail::on_motion_fn<Pose> on_motion;

}  // namespace beluga::policies

#endif
