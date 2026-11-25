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

#include <string>
#include <vector>

#include <opencv2/features2d.hpp>

#include <beluga_visual_odometry/types.hpp>

namespace beluga_visual_odometry {

struct StereoFrame {
  double timestamp{0.0};
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<cv::Point3f> points;
  std::vector<bool> has_point;
};

struct MotionEstimate {
  TrackingStatus status{TrackingStatus::kInvalidInput};
  bool motion_valid{false};
  bool covariance_valid{false};
  // Sensor motion from the previous frame to the current frame. The PnP
  // transform is inverted before it is stored here.
  Sophus::SE3d relative_pose{Sophus::SE3d()};
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};
  std::size_t inlier_count{0};
  std::string failure_reason;
};

}  // namespace beluga_visual_odometry
