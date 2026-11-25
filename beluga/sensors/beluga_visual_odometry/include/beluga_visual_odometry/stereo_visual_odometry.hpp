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

#include <memory>

#include <opencv2/core.hpp>

#include <beluga_visual_odometry/types.hpp>

namespace beluga_visual_odometry {

class StereoVisualOdometry final {
 public:
  explicit StereoVisualOdometry(VisualOdometryConfig config);
  ~StereoVisualOdometry();

  StereoVisualOdometry(StereoVisualOdometry&&) noexcept;
  StereoVisualOdometry& operator=(StereoVisualOdometry&&) noexcept;
  StereoVisualOdometry(const StereoVisualOdometry&) = delete;
  StereoVisualOdometry& operator=(const StereoVisualOdometry&) = delete;

  RelativePlanarMotion Process(
      const cv::Mat& left_image,
      const cv::Mat& right_image,
      double timestamp);
  void Reset();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace beluga_visual_odometry
