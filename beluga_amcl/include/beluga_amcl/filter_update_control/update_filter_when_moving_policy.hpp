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

#ifndef BELUGA_AMCL__FILTER_UPDATE_CONTROL__UPDATE_FILTER_WHEN_MOVING_POLICY_HPP_
#define BELUGA_AMCL__FILTER_UPDATE_CONTROL__UPDATE_FILTER_WHEN_MOVING_POLICY_HPP_

#include <optional>

#include <sophus/se2.hpp>

/**
 * \file
 * \brief Implementation of the resample-on-motion algorithm for resampling.
 */

namespace beluga_amcl
{

/// Parameters used to construct a UpdateFilterWhenMovingPolicy instance.
struct UpdateFilterWhenMovingPolicyParam
{
  /// Distance threshold along x an y (independently) to trigger a resample.
  double update_min_d{0.};
  /// Angular threshold to trigger a resample.
  double update_min_a{0.};
};

/// Implementation of the Resample-On-Motion algorithm for resampling.
class UpdateFilterWhenMovingPolicy
{
public:
  /// Parameter type that the constructor uses to configure the policy.
  using param_type = UpdateFilterWhenMovingPolicyParam;
  /// Type used to exchange and store motion updates by the motion model.
  using motion_event = Sophus::SE2d;

  /// \brief Constructs a UpdateFilterWhenMovingPolicy instance.
  /**
   * \param configuration Policy configuration data.
   */
  explicit UpdateFilterWhenMovingPolicy(const param_type & configuration)
  : configuration_{configuration} {}

  /// \brief Vote whether a filter update must be performed.
  /**
   * \param current_pose_in_odom Current pose of the robot in the odom frame.
   */
  [[nodiscard]] bool do_filter_update(const motion_event & current_pose_in_odom)
  {
    // Don't update the filter unless we've moved far enough from the latest pose
    // where an update was performed. It's an approximation of the recommendations
    // Probabilistic Robotics \cite thrun2005probabilistic Chapter 4.2.4 based
    // in Nav2 AMCL's implementation .

    if (!latest_resample_pose_) {
      // we don't have a previous pose, default to updating the filter
      latest_resample_pose_ = current_pose_in_odom;
      return true;
    }

    // calculate relative transform between previous pose and the current one
    const auto delta = latest_resample_pose_->inverse() * current_pose_in_odom;

    // only resample if movement is above thresholds
    const bool delta_is_above_threshold =  //
      std::abs(delta.translation().x()) > configuration_.update_min_d ||
      std::abs(delta.translation().y()) > configuration_.update_min_d ||
      std::abs(delta.so2().log()) > configuration_.update_min_a;

    if (delta_is_above_threshold) {
      latest_resample_pose_ = current_pose_in_odom;
    }

    return delta_is_above_threshold;
  }

private:
  param_type configuration_;
  std::optional<motion_event> latest_resample_pose_;
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__FILTER_UPDATE_CONTROL__UPDATE_FILTER_WHEN_MOVING_POLICY_HPP_
