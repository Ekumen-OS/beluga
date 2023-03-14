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

#ifndef BELUGA_RESAMPLING_POLICIES_RESAMPLE_ON_MOTION_POLICY_HPP
#define BELUGA_RESAMPLING_POLICIES_RESAMPLE_ON_MOTION_POLICY_HPP

#include <optional>

#include <sophus/se2.hpp>

/**
 * \file
 * \brief Implementation of the resample-on-motion algorithm for resampling.
 */

namespace beluga {

/// Parameters used to construct a ResampleOnMotionPolicy instance.
struct ResampleOnMotionPolicyParam {
  /// Distance threshold along x an y (independently) to trigger a resample.
  double update_min_d{0.};
  /// Angular threshold to trigger a resample.
  double update_min_a{0.};
};

/// Implementation of the Resample-On-Motion algorithm for resampling.
/**
 * ResampleOnMotionPolicy is an implementation of the \ref ResamplingPolicyPage "ResamplingPolicy" named requirements.
 * */
class ResampleOnMotionPolicy {
 public:
  /// Parameter type that the constructor uses to configure the policy.
  using param_type = ResampleOnMotionPolicyParam;
  /// Type used to exchange and store motion updates by the motion model.
  using motion_event = Sophus::SE2d;

  /// Constructs a ResampleOnMotionPolicy instance.
  /**
   * \param configuration Policy configuration data.
   */
  explicit ResampleOnMotionPolicy(const param_type& configuration) : configuration_{configuration} {}

  /// Vote whether resampling must be done according to this policy.
  /**
   * \tparam Concrete Type representing the concrete implementation of the filter.
   * It must satisfy the \ref MotionModelPage MotionModel requirements.
   */
  template <typename Concrete>
  [[nodiscard]] bool do_resampling(Concrete& filter) {
    // To avoid loss of diversity in the particle population, don't
    // resample when the state is known to be static.
    // Probabilistic Robotics \cite thrun2005probabilistic Chapter 4.2.4.
    auto current_pose = filter.latest_motion_update();

    // default to letting other policies decide
    bool must_do_resample{true};

    if (current_pose && latest_resample_pose_) {
      // calculate relative transform between previous pose and the current one
      const auto delta = latest_resample_pose_->inverse() * current_pose.value();

      // only resample if movement is above thresholds
      must_do_resample =  //
          std::abs(delta.translation().x()) > configuration_.update_min_d ||
          std::abs(delta.translation().y()) > configuration_.update_min_d ||
          std::abs(delta.so2().log()) > configuration_.update_min_a;
    }

    // we always measure the distance traveled since the last time we did resample
    if (must_do_resample) {
      latest_resample_pose_ = current_pose;
    }

    return must_do_resample;
  }

 private:
  param_type configuration_;
  std::optional<motion_event> latest_resample_pose_;
};

}  // namespace beluga

#endif
