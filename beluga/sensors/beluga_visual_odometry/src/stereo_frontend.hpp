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

#include <opencv2/features2d.hpp>

#include "types_internal.hpp"

namespace beluga_visual_odometry {

class StereoFrontend final {
 public:
  explicit StereoFrontend(const VisualOdometryConfig& config);
  StereoFrame Process(
      const cv::Mat& left_image,
      const cv::Mat& right_image,
      double timestamp) const;
  const cv::Mat& ProjectionLeft() const { return projection_left_; }
  const cv::Mat& ProjectionRight() const { return projection_right_; }

 private:
  VisualOdometryConfig config_;
  cv::Ptr<cv::ORB> orb_;
  cv::Mat left_map_x_, left_map_y_, right_map_x_, right_map_y_;
  cv::Mat projection_left_, projection_right_;
};

}  // namespace beluga_visual_odometry
