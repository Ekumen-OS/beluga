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

/**
 * @file
 * @brief Includes transform interface declarations.
 */

#include <chrono>
#include <cmath>
#include <optional>
#include <string_view>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation {

/**
 * @brief Abstract interface for querying and publishing coordinate transforms.
 *
 * Provides a middleware-agnostic way to handle spatial frame transformations.
 */
class TransformInterface {
 public:
  virtual ~TransformInterface() = default;

  /// @brief Gets the transform from source_frame to target_frame at a specific time.
  /// @param target_frame The frame to which the transform is requested.
  /// @param source_frame The frame from which the transform is requested.
  /// @param time The time at which the transform is requested. Setting this to a zero timepoint retrieves the latest
  /// available transform.
  /// @param timeout An optional duration to block and wait for the transform to become available.
  /// @return The coordinate transform, or `std::nullopt` if it could not be found.
  virtual std::optional<beluga_estimation::SE3d> LookupTransform(
      std::string_view target_frame,
      std::string_view source_frame,
      TimePoint time,
      std::optional<std::chrono::nanoseconds> timeout = std::nullopt) const = 0;

  /// @brief Gets the transform from source_frame to target_frame at a specific time as an SE2 transform.
  /// @param target_frame The frame to which the transform is requested.
  /// @param source_frame The frame from which the transform is requested.
  /// @param time The time at which the transform is requested. Setting this to a zero timepoint retrieves the latest
  /// available transform.
  /// @param timeout An optional duration to block and wait for the transform to become available.
  /// @return The coordinate transform, or `std::nullopt` if it could not be found.
  std::optional<beluga_estimation::SE2d> LookupTransformSE2(
      std::string_view target_frame,
      std::string_view source_frame,
      TimePoint time,
      std::optional<std::chrono::nanoseconds> timeout = std::nullopt) const {
    const auto transform = LookupTransform(target_frame, source_frame, time, timeout);
    if (!transform) {
      return std::nullopt;
    }
    // Assuming a pure Z-axis rotation (no roll or pitch), R(1,0) = sin(yaw) and R(0,0) = cos(yaw).
    const double yaw = std::atan2(transform->rotationMatrix()(1, 0), transform->rotationMatrix()(0, 0));
    return beluga_estimation::SE2d{yaw, transform->translation().head<2>()};
  }

  /// @brief Gets the transform from source_frame to target_frame at a specific time as an SO3 transform.
  /// @param target_frame The frame to which the transform is requested.
  /// @param source_frame The frame from which the transform is requested.
  /// @param time The time at which the transform is requested. Setting this to a zero timepoint retrieves the latest
  /// available transform.
  /// @param timeout An optional duration to block and wait for the transform to become available.
  /// @return The coordinate transform, or `std::nullopt` if it could not be found.
  std::optional<beluga_estimation::SO3d> LookupTransformSO3(
      std::string_view target_frame,
      std::string_view source_frame,
      TimePoint time,
      std::optional<std::chrono::nanoseconds> timeout = std::nullopt) const {
    const auto transform = LookupTransform(target_frame, source_frame, time, timeout);
    if (!transform) {
      return std::nullopt;
    }
    return transform->so3();
  }

  /// @brief Gets the transform from source_frame to target_frame at a specific time as an SO2 transform.
  /// @param target_frame The frame to which the transform is requested.
  /// @param source_frame The frame from which the transform is requested.
  /// @param time The time at which the transform is requested. Setting this to a zero timepoint retrieves the latest
  /// available transform.
  /// @param timeout An optional duration to block and wait for the transform to become available.
  /// @return The coordinate transform, or `std::nullopt` if it could not be found.
  std::optional<beluga_estimation::SO2d> LookupTransformSO2(
      std::string_view target_frame,
      std::string_view source_frame,
      TimePoint time,
      std::optional<std::chrono::nanoseconds> timeout = std::nullopt) const {
    const auto transform = LookupTransform(target_frame, source_frame, time, timeout);
    if (!transform) {
      return std::nullopt;
    }
    // Assuming a pure Z-axis rotation (no roll or pitch), R(1,0) = sin(yaw) and R(0,0) = cos(yaw).
    const double yaw = std::atan2(transform->rotationMatrix()(1, 0), transform->rotationMatrix()(0, 0));
    return beluga_estimation::SO2d{yaw};
  }

  /// @brief Publishes a transform from source_frame to target_frame.
  /// @param target_frame The frame to which the transform is published.
  /// @param source_frame The frame from which the transform is published.
  /// @param time The time at which the transform is published.
  /// @param transform The coordinate transform to publish.
  virtual void BroadcastTransform(
      std::string_view target_frame,
      std::string_view source_frame,
      TimePoint time,
      const beluga_estimation::SE3d& transform) = 0;

  /// @brief Publishes an SE2 transform from source_frame to target_frame.
  /// @param target_frame The frame to which the transform is published.
  /// @param source_frame The frame from which the transform is published.
  /// @param time The time at which the transform is published.
  /// @param transform The coordinate transform to publish.
  void BroadcastTransform(
      std::string_view target_frame,
      std::string_view source_frame,
      TimePoint time,
      const beluga_estimation::SE2d& transform) {
    BroadcastTransform(
        target_frame, source_frame, time,
        beluga_estimation::SE3d{
            Eigen::Quaterniond{Eigen::AngleAxisd{transform.so2().log(), Eigen::Vector3d::UnitZ()}},
            Eigen::Vector3d{transform.translation().x(), transform.translation().y(), 0.0}});
  }

  /// @brief Publishes an SO3 transform from source_frame to target_frame.
  /// @param target_frame The frame to which the transform is published.
  /// @param source_frame The frame from which the transform is published.
  /// @param time The time at which the transform is published.
  /// @param transform The coordinate transform to publish.
  void BroadcastTransform(
      std::string_view target_frame,
      std::string_view source_frame,
      TimePoint time,
      const beluga_estimation::SO3d& transform) {
    BroadcastTransform(target_frame, source_frame, time, beluga_estimation::SE3d{transform, Eigen::Vector3d::Zero()});
  }

  /// @brief Publishes an SO2 transform from source_frame to target_frame.
  /// @param target_frame The frame to which the transform is published.
  /// @param source_frame The frame from which the transform is published.
  /// @param time The time at which the transform is published.
  /// @param transform The coordinate transform to publish.
  void BroadcastTransform(
      std::string_view target_frame,
      std::string_view source_frame,
      TimePoint time,
      const beluga_estimation::SO2d& transform) {
    BroadcastTransform(
        target_frame, source_frame, time,
        beluga_estimation::SE3d{
            Eigen::Quaterniond{Eigen::AngleAxisd{transform.log(), Eigen::Vector3d::UnitZ()}}, Eigen::Vector3d::Zero()});
  }
};

}  // namespace beluga_estimation
