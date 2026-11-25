// Copyright 2026 Ekumen, Inc.
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

#include <beluga_visual_odometry/types.hpp>

namespace beluga_visual_odometry {

class PlanarProjector final {
 public:
  explicit PlanarProjector(const VisualOdometryConfig& config) : config_(config) {}
  // Projects a relative camera motion into the base-frame X/Z ground plane.
  // This contract is different from the ROS adapter's absolute world-frame
  // X/Y odometry projection, so the standalone runner does not depend on ROS.
  Eigen::Vector3d Project(const Sophus::SE3d& relative_pose) const;
  bool ProjectCovariance(
      const Sophus::SE3d&,
      const Eigen::Matrix<double, 6, 6>&,
      Eigen::Matrix3d&) const;

 private:
  VisualOdometryConfig config_;
};

}  // namespace beluga_visual_odometry
