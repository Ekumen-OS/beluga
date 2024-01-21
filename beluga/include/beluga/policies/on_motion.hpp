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

namespace beluga::policies {

namespace detail {

/// Implementation detail for an on_motion policy.
template <class Scalar>
struct on_motion_policy {
 public:
  /// Constructor.
  constexpr on_motion_policy(double min_distance, double min_angle)
      : min_distance_(min_distance), min_angle_(min_angle) {}

  /// Call operator overload.
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

 public:
  double min_distance_{0.0};
  double min_angle_{0.0};
  std::optional<Sophus::SE2<Scalar>> latest_pose_;
};

/// Implementation detail for an on_motion_fn object.
struct on_motion_fn {
  /// Overload that creates the policy closure.
  template <class Scalar>
  constexpr auto operator()(Scalar min_distance, Scalar min_angle) const {
    return beluga::make_policy_closure(on_motion_policy<Scalar>{min_distance, min_angle});
  }
};

}  // namespace detail

/// Policy that can be used to trigger on motion.
inline constexpr detail::on_motion_fn on_motion;

}  // namespace beluga::policies

#endif
