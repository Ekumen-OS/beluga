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

#ifndef BELUGA_SENSOR_BEARING_SENSOR_MODEL_HPP
#define BELUGA_SENSOR_BEARING_SENSOR_MODEL_HPP

// external
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

// standard library
#include <cmath>
#include <random>
#include <utility>
#include <vector>

// project
#include <beluga/types/landmark_detection_types.hpp>

/**
 * \file
 * \brief Implementation of a discrete landmark bearing sensor model.
 */

namespace beluga {

/// Parameters used to construct a BearingSensorModel instance.
struct BearingModelParam {
  double sigma_bearing{1.0};            ///< Standard deviation of the bearing error.
  Sophus::SE3d sensor_pose_in_robot{};  ///< Pose of the sensor in the robot reference frame.
};

/// Generic bearing sensor model, for both 2D and 3D state types.
/**
 * This class satisfies \ref SensorModelPage.
 * \tparam LandmarkMap class managing the list of known landmarks.
 * \tparam StateType type of the state of the particle.
 */
template <class LandmarkMap, class StateType>
class BearingSensorModel {
 public:
  /// State type of a particle.
  using state_type = StateType;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: vector of landmark detections
  using measurement_type = std::vector<LandmarkBearingDetection>;
  /// Map representation type.
  using map_type = LandmarkMap;
  /// Parameter type that the constructor uses to configure the bearing sensor model.
  using param_type = BearingModelParam;

  /// Constructs a BearingSensorModel instance.
  /**
   * \param params Parameters to configure this instance. See beluga::BearingModelParam for details.
   * \param landmark_map Map of landmarks to be used by the sensor model to compute importance weights
   *  for particle states.
   */
  template <class... Args>
  explicit BearingSensorModel(param_type params, LandmarkMap landmark_map)
      : params_{std::move(params)}, landmark_map_{std::move(landmark_map)} {}

  /// Returns a state weighting function conditioned on landmark bearing detections.
  /**
   * \param detections Landmark bearing detections in the reference frame of particle states.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   *  and borrowing a reference to this sensor model (and thus their lifetime are bound).
   */
  [[nodiscard]] auto operator()(measurement_type&& detections) const {
    return [this, detections = std::move(detections)](const state_type& state) -> weight_type {
      Sophus::SE3d robot_pose_in_world;

      if constexpr (std::is_same_v<state_type, Sophus::SE3d>) {
        // The robot pose state is already given in 3D,
        robot_pose_in_world = state;
      } else {
        // The robot pose state is given in 2D. Notice that in this case
        // the 2D pose of the robot is assumed to be that of the robot footprint (projection of the robot
        // on the z=0 plane of the 3D world frame). This is so that we can tie the sensor reference frame
        // to the world frame where the landmarks are given without additional structural information.
        robot_pose_in_world = Sophus::SE3d{
            Sophus::SO3d::rotZ(state.so2().log()),
            Eigen::Vector3d{state.translation().x(), state.translation().y(), 0.0}};
      }

      // precalculate the sensor pose in the world frame
      const auto sensor_pose_in_world = robot_pose_in_world * params_.sensor_pose_in_robot;

      const auto detection_weight = [this, &sensor_pose_in_world](const auto& detection) {
        // find the landmark the most closely matches the sample bearing vector
        const auto opt_landmark_bearing_in_sensor = landmark_map_.find_closest_bearing_landmark(
            detection.detection_bearing_in_sensor, detection.category, sensor_pose_in_world);

        // if we did not find a matching landmark, return 0.0
        if (!opt_landmark_bearing_in_sensor) {
          return 0.0;
        }

        // recover sample bearing vector
        const auto detection_bearing_in_sensor = detection.detection_bearing_in_sensor.normalized();

        // recover landmark bearing vector
        const auto& landmark_bearing_in_sensor = *opt_landmark_bearing_in_sensor;

        // calculate the aperture angle between the detection and the landmark
        const auto cos_aperture = detection_bearing_in_sensor.dot(landmark_bearing_in_sensor);
        const auto sin_aperture = detection_bearing_in_sensor.cross(landmark_bearing_in_sensor).norm();

        // calculate the angle between the sample and the landmark
        const auto bearing_error = std::atan2(sin_aperture, cos_aperture);

        // model the probability of the landmark being detected as depending on the bearing error
        const auto bearing_error_prob =
            std::exp(-bearing_error * bearing_error / (2. * params_.sigma_bearing * params_.sigma_bearing));

        // We'll assume the probability of identification error to be zero
        const auto prob = bearing_error_prob;

        // TODO(unknown): We continue to use the sum-of-cubes formula that nav2 uses
        // See https://github.com/Ekumen-OS/beluga/issues/153
        return prob * prob * prob;
      };

      return std::transform_reduce(detections.cbegin(), detections.cend(), 1.0, std::plus{}, detection_weight);
    };
  }

  /// Update the sensor model with a new landmark `map`.
  void update_map(map_type&& map) { landmark_map_ = std::move(map); }

 private:
  param_type params_;
  LandmarkMap landmark_map_;
};

/// Sensor model based on discrete landmarks bearing detection for 2D state types.
template <class LandmarkMap>
using BearingSensorModel2d = BearingSensorModel<LandmarkMap, Sophus::SE2d>;

/// Sensor model based on discrete landmarks bearing detection for 3D state types.
template <class LandmarkMap>
using BearingSensorModel3d = BearingSensorModel<LandmarkMap, Sophus::SE3d>;

}  // namespace beluga

#endif
