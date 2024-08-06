// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_SENSOR_DATA_LANDMARK_MAP_HPP
#define BELUGA_SENSOR_DATA_LANDMARK_MAP_HPP

// external
#include <range/v3/view/filter.hpp>
#include <range/v3/view/tail.hpp>
#include <sophus/se3.hpp>

// standard library
#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

// project
#include <beluga/types/landmark_detection_types.hpp>

/**
 * \file
 * \brief Landmark map datatype.
 */

namespace beluga {

/// Basic 3D landmark map datatype
class LandmarkMap {
 public:
  /// Vector of landmarks
  using landmarks_set_position_data = std::vector<LandmarkPositionDetection>;
  /// Type used to represent poses in the world frame
  using world_pose_type = Sophus::SE3d;

  /// @brief Constructor.
  /// @param boundaries Limits of the map.
  /// @param landmarks List of landmarks that can be expected to be detected.
  explicit LandmarkMap(const LandmarkMapBoundaries& boundaries, landmarks_set_position_data landmarks)
      : landmarks_(std::move(landmarks)), map_boundaries_(std::move(boundaries)) {}

  /// @brief Constructor with implicit map boundaries (computed from landmarks).
  /// @details Note that computing map boundaries from landmarks will effectively
  /// constrain the traversable region to the volume delimited by such landmarks.
  /// In many cases this is undesirable e.g. when landmarks are scattered in some
  /// region of space, when landmarks are distributed on some flat surface such
  /// as a wall, when landmarks are located within a small area at some height
  /// for visibility, etc. Use with care.
  /// @param landmarks List of landmarks that can be expected to be detected.
  explicit LandmarkMap(landmarks_set_position_data landmarks) : landmarks_(std::move(landmarks)) {
    if (!landmarks_.empty()) {
      map_boundaries_.min() = landmarks_[0].detection_position_in_robot;
      map_boundaries_.max() = landmarks_[0].detection_position_in_robot;
      for (const auto& landmark : ranges::views::tail(landmarks_)) {
        const auto& position = landmark.detection_position_in_robot;
        map_boundaries_.min() = map_boundaries_.min().cwiseMin(position);
        map_boundaries_.max() = map_boundaries_.max().cwiseMax(position);
      }
    }
  }

  /// @brief Returns the map boundaries.
  /// @return The map boundaries.
  [[nodiscard]] LandmarkMapBoundaries map_limits() const { return map_boundaries_; }

  /// @brief Finds the nearest landmark to a given detection and returns it's data.
  /// @param detection_position_in_world The detection data.
  /// @param detection_category The category of the detection.
  /// @return The landmark data. nullopt if no landmark was found.
  [[nodiscard]] std::optional<LandmarkPosition3> find_nearest_landmark(
      const LandmarkPosition3& detection_position_in_world,
      const LandmarkCategory& detection_category) const {
    // only consider those that have the same id
    auto same_category_landmarks_view =
        landmarks_ | ranges::views::filter([detection_category = detection_category](const auto& l) {
          return detection_category == l.category;
        });

    // find the landmark that minimizes the distance to the detection position
    // This is O(n). A spatial data structure should be used instead.
    auto min = std::min_element(
        same_category_landmarks_view.begin(), same_category_landmarks_view.end(),
        [&detection_position_in_world](const auto& a, const auto& b) {
          const auto& landmark_a_position_in_world = a.detection_position_in_robot;
          const auto& landmark_b_position_in_world = b.detection_position_in_robot;

          const auto landmark_b_squared_in_world_squared =
              (landmark_a_position_in_world - detection_position_in_world).squaredNorm();
          const auto landmark_b_distance_in_world_squared =
              (landmark_b_position_in_world - detection_position_in_world).squaredNorm();
          return landmark_b_squared_in_world_squared < landmark_b_distance_in_world_squared;
        });

    if (min == same_category_landmarks_view.end()) {
      return std::nullopt;
    }

    return min->detection_position_in_robot;
  }

  /// @brief Finds the landmark that minimizes the bearing error to a given detection and returns its data.
  /// @param detection_bearing_in_sensor The detection data.
  /// @param detection_category The category of the detection.
  /// @param sensor_pose_in_world The pose of the sensor in the world frame.
  /// @return The landmark data. nullopt if no landmark was found.
  [[nodiscard]] std::optional<LandmarkBearing3> find_closest_bearing_landmark(
      const LandmarkBearing3& detection_bearing_in_sensor,
      const LandmarkCategory& detection_category,
      const world_pose_type& sensor_pose_in_world) const {
    // only consider those that have the same detection id (category)
    auto same_category_landmarks_view =
        landmarks_ | ranges::views::filter([detection_category = detection_category](const auto& l) {
          return detection_category == l.category;
        });

    // find the landmark that minimizes the bearing error
    const auto world_in_sensor_transform = sensor_pose_in_world.inverse();

    // This whole search thing is very expensive, with objects getting created, normalized and
    // destroyed multiple times and the same transformations being calculated over and over again.
    // This will only work as a proof-of-concept, but it needs to be optimized for large numbers of
    // landmarks.
    const auto minimization_function = [&detection_bearing_in_sensor, &world_in_sensor_transform](
                                           const auto& a, const auto& b) {
      const auto& landmark_a_position_in_world = a.detection_position_in_robot;
      const auto& landmark_b_position_in_world = b.detection_position_in_robot;

      // convert the landmark locations relative to the sensor frame
      const auto landmark_a_bearing_in_sensor = (world_in_sensor_transform * landmark_a_position_in_world).normalized();
      const auto landmark_b_bearing_in_sensor = (world_in_sensor_transform * landmark_b_position_in_world).normalized();

      // find the landmark that minimizes the bearing error by maximizing the dot product against the
      // detection bearing vector
      const auto dot_product_a = landmark_a_bearing_in_sensor.dot(detection_bearing_in_sensor);
      const auto dot_product_b = landmark_b_bearing_in_sensor.dot(detection_bearing_in_sensor);

      return dot_product_a > dot_product_b;
    };

    auto min = std::min_element(
        same_category_landmarks_view.begin(), same_category_landmarks_view.end(), minimization_function);

    if (min == same_category_landmarks_view.end()) {
      return std::nullopt;
    }

    // find the normalized bearing vector to the landmark, relative to the sensor frame
    const auto landmark_position_in_sensor = world_in_sensor_transform * min->detection_position_in_robot;
    return landmark_position_in_sensor.normalized();
  }

 private:
  landmarks_set_position_data landmarks_;
  LandmarkMapBoundaries map_boundaries_;
};

}  // namespace beluga

#endif
