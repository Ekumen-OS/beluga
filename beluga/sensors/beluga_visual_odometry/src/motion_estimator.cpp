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

#include <Eigen/SVD>
#include <opencv2/calib3d.hpp>

#include "matching.hpp"
#include "motion_estimator.hpp"

namespace beluga_visual_odometry {
namespace {

constexpr double kFiniteDifferenceStep = 1.0e-6;
constexpr double kMinimumSingularValueRatio = 1.0e-8;

Sophus::SE3d ToSophus(const cv::Mat& rotation, const cv::Mat& translation) {
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  for (int row = 0; row < 3; ++row) {
    t(row) = translation.at<double>(row, 0);
    for (int column = 0; column < 3; ++column) {
      r(row, column) = rotation.at<double>(row, column);
    }
  }
  return Sophus::SE3d(Sophus::SO3d(r), t);
}

Eigen::Vector2d Project(
    const cv::Mat& camera_matrix,
    const Sophus::SE3d& pose,
    const cv::Point3f& point) {
  const Eigen::Vector3d p = pose * Eigen::Vector3d(point.x, point.y, point.z);
  return {
      camera_matrix.at<double>(0, 0) * p.x() / p.z() + camera_matrix.at<double>(0, 2),
      camera_matrix.at<double>(1, 1) * p.y() / p.z() + camera_matrix.at<double>(1, 2)};
}

bool InvertPoseCovariance(
    const Sophus::SE3d& pose,
    const Eigen::Matrix<double, 6, 6>& covariance,
    Eigen::Matrix<double, 6, 6>& inverse_covariance) {
  const Sophus::SE3d inverse_pose = pose.inverse();
  Eigen::Matrix<double, 6, 6> jacobian;
  for (int column = 0; column < 6; ++column) {
    Eigen::Matrix<double, 6, 1> perturbation =
        Eigen::Matrix<double, 6, 1>::Zero();
    perturbation(column) = kFiniteDifferenceStep;
    const Sophus::SE3d perturbed_inverse =
        (Sophus::SE3d::exp(perturbation) * pose).inverse();
    const Eigen::Matrix<double, 6, 1> delta =
        (perturbed_inverse * inverse_pose.inverse()).log();
    jacobian.col(column) = delta / kFiniteDifferenceStep;
  }
  if (!jacobian.allFinite()) {
    return false;
  }
  inverse_covariance = jacobian * covariance * jacobian.transpose();
  inverse_covariance =
      0.5 * (inverse_covariance + inverse_covariance.transpose());
  return inverse_covariance.allFinite();
}

}  // namespace

MotionEstimator::MotionEstimator(
    const VisualOdometryConfig& config,
    const cv::Mat& projection_left)
    : config_(config), projection_left_(projection_left.clone()) {
  if (config.pixel_noise_sigma <= 0.0 || config.descriptor_ratio <= 0.0) {
    throw std::invalid_argument("Invalid motion-estimator configuration");
  }
}

MotionEstimate MotionEstimator::Estimate(
    const StereoFrame& previous,
    const StereoFrame& current) const {
  MotionEstimate result;
  const auto matches = MatchDescriptors(
      previous.descriptors, current.descriptors, config_.descriptor_ratio);
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  for (const auto& match : matches) {
    const auto previous_index = static_cast<std::size_t>(match.queryIdx);
    const auto current_index = static_cast<std::size_t>(match.trainIdx);
    if (previous_index < previous.has_point.size() &&
        current_index < current.keypoints.size() && previous.has_point[previous_index]) {
      object_points.push_back(previous.points[previous_index]);
      image_points.push_back(current.keypoints[current_index].pt);
    }
  }
  if (object_points.size() < 4) {
    result.status = TrackingStatus::kInsufficientFeatures;
    result.failure_reason = "fewer than four frame-to-frame correspondences";
    return result;
  }

  const cv::Mat camera_matrix = projection_left_(cv::Rect(0, 0, 3, 3));
  const cv::Mat zero_distortion = cv::Mat::zeros(1, 5, CV_64F);
  cv::Mat rotation_vector, translation_vector, inlier_indices;
  const bool pnp_ok = cv::solvePnPRansac(
      object_points,
      image_points,
      camera_matrix,
      zero_distortion,
      rotation_vector,
      translation_vector,
      false,
      config_.pnp_iterations,
      static_cast<float>(config_.pnp_reprojection_error_pixels),
      config_.pnp_confidence,
      inlier_indices,
      cv::SOLVEPNP_EPNP);
  if (!pnp_ok) {
    result.status = TrackingStatus::kInsufficientInliers;
    result.failure_reason = "solvePnPRansac failed";
    return result;
  }

  std::vector<cv::Point3f> inlier_object_points;
  std::vector<cv::Point2f> inlier_image_points;
  for (int row = 0; row < inlier_indices.rows; ++row) {
    const int index = inlier_indices.at<int>(row, 0);
    if (index >= 0 && static_cast<std::size_t>(index) < object_points.size()) {
      inlier_object_points.push_back(object_points[static_cast<std::size_t>(index)]);
      inlier_image_points.push_back(image_points[static_cast<std::size_t>(index)]);
    }
  }
  result.inlier_count = inlier_object_points.size();
  if (result.inlier_count < static_cast<std::size_t>(config_.min_inliers)) {
    result.status = TrackingStatus::kInsufficientInliers;
    result.failure_reason = "insufficient PnP inliers";
    return result;
  }

  cv::solvePnPRefineLM(
      inlier_object_points,
      inlier_image_points,
      camera_matrix,
      zero_distortion,
      rotation_vector,
      translation_vector);
  cv::Mat rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix);
  rotation_matrix.convertTo(rotation_matrix, CV_64F);
  translation_vector.convertTo(translation_vector, CV_64F);
  const Sophus::SE3d pnp_pose = ToSophus(rotation_matrix, translation_vector);

  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  const double inverse_variance = 1.0 /
      (config_.pixel_noise_sigma * config_.pixel_noise_sigma);
  bool valid = true;
  for (const auto& point : inlier_object_points) {
    const Eigen::Vector2d nominal = Project(camera_matrix, pnp_pose, point);
    if (!nominal.allFinite()) {
      valid = false;
      continue;
    }
    Eigen::Matrix<double, 2, 6> jacobian;
    for (int column = 0; column < 6; ++column) {
      Eigen::Matrix<double, 6, 1> perturbation =
          Eigen::Matrix<double, 6, 1>::Zero();
      perturbation(column) = kFiniteDifferenceStep;
      const auto perturbed_pose =
          Sophus::SE3d::exp(perturbation) * pnp_pose;
      const Eigen::Vector2d projected = Project(camera_matrix, perturbed_pose, point);
      jacobian(0, column) =
          (projected.x() - nominal.x()) / kFiniteDifferenceStep;
      jacobian(1, column) =
          (projected.y() - nominal.y()) / kFiniteDifferenceStep;
    }
    valid = valid && jacobian.allFinite();
    if (valid) {
      information += inverse_variance * jacobian.transpose() * jacobian;
    }
  }

  const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
      information, Eigen::ComputeFullV);
  const auto singular_values = svd.singularValues();
  const double largest = singular_values.maxCoeff();
  const double smallest = singular_values.minCoeff();
  result.covariance_valid = valid && largest > 0.0 &&
      smallest > largest * kMinimumSingularValueRatio;
  if (result.covariance_valid) {
    Eigen::Matrix<double, 6, 6> pnp_covariance =
        Eigen::Matrix<double, 6, 6>::Zero();
    for (int index = 0; index < 6; ++index) {
      pnp_covariance +=
          (1.0 / singular_values(index)) * svd.matrixV().col(index) *
          svd.matrixV().col(index).transpose();
    }
    result.covariance_valid = InvertPoseCovariance(
        pnp_pose, pnp_covariance, result.covariance);
  }
  result.relative_pose = pnp_pose.inverse();
  result.motion_valid = true;
  result.status = result.covariance_valid
      ? TrackingStatus::kTracking
      : TrackingStatus::kDegenerate;
  if (!result.covariance_valid) {
    result.failure_reason = "degenerate reprojection geometry";
  }
  return result;
}

}  // namespace beluga_visual_odometry
