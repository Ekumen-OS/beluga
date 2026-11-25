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
#include <utility>

#include <beluga_visual_odometry/stereo_visual_odometry.hpp>

#include "motion_estimator.hpp"
#include "planar_projection.hpp"
#include "stereo_frontend.hpp"

namespace beluga_visual_odometry {

struct StereoVisualOdometry::Impl {
  explicit Impl(VisualOdometryConfig config_value)
      : config(std::move(config_value)),
        frontend(config),
        motion_estimator(config, frontend.ProjectionLeft()),
        planar_projector(config) {}

  VisualOdometryConfig config;
  StereoFrontend frontend;
  MotionEstimator motion_estimator;
  PlanarProjector planar_projector;
  std::unique_ptr<StereoFrame> previous_frame;
};

StereoVisualOdometry::StereoVisualOdometry(VisualOdometryConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
  if (impl_->config.min_inliers < 4 || impl_->config.pnp_iterations <= 0 ||
      impl_->config.pnp_confidence <= 0.0 ||
      impl_->config.pnp_confidence >= 1.0 ||
      !std::isfinite(impl_->config.pnp_confidence) ||
      impl_->config.minimum_tracking_ratio < 0.0 ||
      impl_->config.minimum_tracking_ratio > 1.0 ||
      !std::isfinite(impl_->config.minimum_tracking_ratio)) {
    throw std::invalid_argument("Invalid PnP configuration");
  }
}

StereoVisualOdometry::~StereoVisualOdometry() = default;

StereoVisualOdometry::StereoVisualOdometry(StereoVisualOdometry&&) noexcept = default;

StereoVisualOdometry& StereoVisualOdometry::operator=(
    StereoVisualOdometry&&) noexcept = default;

RelativePlanarMotion StereoVisualOdometry::Process(
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    double timestamp) {
  RelativePlanarMotion result;
  result.end_time = timestamp;
  result.sensor_frame = impl_->config.sensor_frame;
  if (!std::isfinite(timestamp) ||
      (impl_->previous_frame && timestamp <= impl_->previous_frame->timestamp)) {
    result.start_time = impl_->previous_frame
        ? impl_->previous_frame->timestamp
        : timestamp;
    result.failure_reason = !std::isfinite(timestamp)
        ? "timestamp is not finite"
        : "timestamp is not strictly increasing";
    return result;
  }

  StereoFrame current;
  try {
    current = impl_->frontend.Process(left_image, right_image, timestamp);
  } catch (const std::exception& exception) {
    result.start_time = impl_->previous_frame
        ? impl_->previous_frame->timestamp
        : timestamp;
    result.failure_reason = exception.what();
    return result;
  }
  if (!impl_->previous_frame) {
    result.start_time = timestamp;
    result.status = TrackingStatus::kInitializing;
    impl_->previous_frame = std::make_unique<StereoFrame>(std::move(current));
    return result;
  }

  result.start_time = impl_->previous_frame->timestamp;
  const MotionEstimate estimate =
      impl_->motion_estimator.Estimate(*impl_->previous_frame, current);
  result.inlier_count = estimate.inlier_count;
  result.failure_reason = estimate.failure_reason;
  if (!estimate.motion_valid) {
    result.status = estimate.status;
    impl_->previous_frame = std::make_unique<StereoFrame>(std::move(current));
    return result;
  }

  result.delta = impl_->planar_projector.Project(estimate.relative_pose);
  result.status = estimate.status;
  if (!estimate.covariance_valid || !impl_->planar_projector.ProjectCovariance(
          estimate.relative_pose, estimate.covariance, result.covariance)) {
    result.status = TrackingStatus::kDegenerate;
    result.failure_reason = "invalid planar covariance";
  }
  impl_->previous_frame = std::make_unique<StereoFrame>(std::move(current));
  return result;
}

void StereoVisualOdometry::Reset() {
  impl_->previous_frame.reset();
}

}  // namespace beluga_visual_odometry
