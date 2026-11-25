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

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace beluga_visual_odometry {

enum class TrackingStatus : std::uint8_t {
  kInitializing,
  kTracking,
  kInsufficientFeatures,
  kInsufficientInliers,
  kDegenerate,
  kInvalidInput,
};

struct StereoCalibration {
  cv::Size image_size;
  cv::Mat left_camera_matrix;
  cv::Mat left_distortion;
  cv::Mat right_camera_matrix;
  cv::Mat right_distortion;
  cv::Mat rotation_right_left;
  cv::Mat translation_right_left;

  static StereoCalibration LoadFromFile(const std::string& path);
};

struct VisualOdometryConfig {
  StereoCalibration calibration;
  int orb_features{1200};
  float orb_scale_factor{1.2F};
  int orb_levels{8};
  int orb_fast_threshold{20};
  double descriptor_ratio{0.75};
  double max_vertical_error_pixels{2.0};
  double min_disparity_pixels{1.0};
  double min_depth_meters{0.2};
  double max_depth_meters{50.0};
  double pnp_reprojection_error_pixels{3.0};
  int pnp_iterations{100};
  double pnp_confidence{0.999};
  int min_inliers{12};
  double pixel_noise_sigma{1.0};
  double minimum_tracking_ratio{0.0};
  Sophus::SE3d T_base_camera{Sophus::SE3d()};
  std::string sensor_frame{"camera_left"};
};

struct RelativePlanarMotion {
  double start_time{0.0};
  double end_time{0.0};
  std::string sensor_frame{"camera_left"};
  Eigen::Vector3d delta{
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Matrix3d covariance{
      Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  std::size_t inlier_count{0};
  TrackingStatus status{TrackingStatus::kInvalidInput};
  std::string failure_reason;
};

VisualOdometryConfig LoadConfiguration(const std::string& path);

}  // namespace beluga_visual_odometry
