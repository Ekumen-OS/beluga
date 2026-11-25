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

#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "matching.hpp"
#include "stereo_frontend.hpp"

namespace beluga_visual_odometry {
namespace {

constexpr double kDepthEpsilon = 1.0e-9;

}  // namespace

std::vector<cv::DMatch> MatchDescriptors(
    const cv::Mat& query,
    const cv::Mat& train,
    double ratio) {
  std::vector<cv::DMatch> matches;
  if (query.empty() || train.empty() || query.rows < 2 || train.rows < 2) {
    return matches;
  }
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  const auto ratio_matches = [&](const cv::Mat& source, const cv::Mat& target) {
    std::vector<cv::DMatch> result;
    std::vector<std::vector<cv::DMatch>> pairs;
    matcher.knnMatch(source, target, pairs, 2);
    for (const auto& pair : pairs) {
      if (pair.size() == 2 &&
          pair[0].distance < static_cast<float>(ratio) * pair[1].distance) {
        result.push_back(pair[0]);
      }
    }
    return result;
  };

  matches = ratio_matches(query, train);
  return matches;
}

StereoFrontend::StereoFrontend(const VisualOdometryConfig& config)
    : config_(config),
      orb_(cv::ORB::create(
          config.orb_features,
          config.orb_scale_factor,
          config.orb_levels,
          31,
          0,
          2,
          cv::ORB::HARRIS_SCORE,
          31,
          config.orb_fast_threshold)) {
  if (config.calibration.image_size.width <= 0 ||
      config.calibration.image_size.height <= 0 ||
      config.calibration.left_camera_matrix.empty() ||
      config.calibration.right_camera_matrix.empty() ||
      config.calibration.rotation_right_left.empty() ||
      config.calibration.translation_right_left.empty() ||
      config.descriptor_ratio <= 0.0) {
    throw std::invalid_argument("Invalid stereo frontend configuration");
  }

  cv::Mat left_rectification, right_rectification, unused_q;
  cv::stereoRectify(
      config.calibration.left_camera_matrix,
      config.calibration.left_distortion,
      config.calibration.right_camera_matrix,
      config.calibration.right_distortion,
      config.calibration.image_size,
      config.calibration.rotation_right_left,
      config.calibration.translation_right_left,
      left_rectification,
      right_rectification,
      projection_left_,
      projection_right_,
      unused_q,
      cv::CALIB_ZERO_DISPARITY,
      0.0);
  cv::initUndistortRectifyMap(
      config.calibration.left_camera_matrix,
      config.calibration.left_distortion,
      left_rectification,
      projection_left_,
      config.calibration.image_size,
      CV_32FC1,
      left_map_x_,
      left_map_y_);
  cv::initUndistortRectifyMap(
      config.calibration.right_camera_matrix,
      config.calibration.right_distortion,
      right_rectification,
      projection_right_,
      config.calibration.image_size,
      CV_32FC1,
      right_map_x_,
      right_map_y_);
  projection_left_.convertTo(projection_left_, CV_64F);
  projection_right_.convertTo(projection_right_, CV_64F);
  const double baseline = std::abs(
      projection_right_.at<double>(0, 3) / projection_right_.at<double>(0, 0));
  if (!std::isfinite(baseline) || baseline <= 0.0) {
    throw std::invalid_argument("Stereo rectification produced an invalid baseline");
  }
}

StereoFrame StereoFrontend::Process(
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    double timestamp) const {
  if (left_image.empty() || right_image.empty() ||
      left_image.channels() != 1 || right_image.channels() != 1 ||
      left_image.size() != config_.calibration.image_size ||
      right_image.size() != config_.calibration.image_size) {
    throw std::invalid_argument("Stereo input must be grayscale and match calibration");
  }

  cv::Mat left_rectified, right_rectified;
  cv::remap(left_image, left_rectified, left_map_x_, left_map_y_, cv::INTER_LINEAR);
  cv::remap(right_image, right_rectified, right_map_x_, right_map_y_, cv::INTER_LINEAR);

  StereoFrame output;
  output.timestamp = timestamp;
  orb_->detectAndCompute(left_rectified, cv::noArray(), output.keypoints, output.descriptors);
  output.points.resize(output.keypoints.size());
  output.has_point.assign(output.keypoints.size(), false);
  if (output.descriptors.empty()) {
    return output;
  }

  std::vector<cv::KeyPoint> right_keypoints;
  cv::Mat right_descriptors;
  orb_->detectAndCompute(right_rectified, cv::noArray(), right_keypoints, right_descriptors);
  const auto matches = MatchDescriptors(
      output.descriptors, right_descriptors, config_.descriptor_ratio);

  std::vector<cv::Point2f> left_points, right_points;
  std::vector<cv::DMatch> valid_matches;
  for (const auto& match : matches) {
    const auto& left = output.keypoints[static_cast<std::size_t>(match.queryIdx)].pt;
    const auto& right = right_keypoints[static_cast<std::size_t>(match.trainIdx)].pt;
    if (std::abs(static_cast<double>(left.y - right.y)) >
            config_.max_vertical_error_pixels ||
        left.x - right.x < config_.min_disparity_pixels) {
      continue;
    }
    left_points.push_back(left);
    right_points.push_back(right);
    valid_matches.push_back(match);
  }
  if (left_points.empty()) {
    return output;
  }

  cv::Mat homogeneous;
  cv::triangulatePoints(
      projection_left_, projection_right_, left_points, right_points, homogeneous);
  homogeneous.convertTo(homogeneous, CV_64F);
  for (int index = 0; index < homogeneous.cols; ++index) {
    const double w = homogeneous.at<double>(3, index);
    if (std::abs(w) <= kDepthEpsilon) {
      continue;
    }
    const cv::Point3f point(
        static_cast<float>(homogeneous.at<double>(0, index) / w),
        static_cast<float>(homogeneous.at<double>(1, index) / w),
        static_cast<float>(homogeneous.at<double>(2, index) / w));
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || point.z < config_.min_depth_meters ||
        point.z > config_.max_depth_meters) {
      continue;
    }
    const int left_index = valid_matches[static_cast<std::size_t>(index)].queryIdx;
    output.points[static_cast<std::size_t>(left_index)] = point;
    output.has_point[static_cast<std::size_t>(left_index)] = true;
  }
  return output;
}

}  // namespace beluga_visual_odometry
