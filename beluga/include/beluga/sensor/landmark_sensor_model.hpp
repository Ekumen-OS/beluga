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

#ifndef BELUGA_SENSOR_LANDMARK_SENSOR_MODEL_HPP
#define BELUGA_SENSOR_LANDMARK_SENSOR_MODEL_HPP

// external
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

// standard library
#include <cmath>
#include <random>
#include <vector>

// project
#include <beluga/types/landmark_detection_types.hpp>

/**
 * \file
 * \brief Implementation of a discrete landmark sensor model.
 */

namespace beluga {

/// Parameters used to construct a LandmarkSensorModel instance (both 2D and 3D).
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic section 6.6
 */
struct LandmarkModelParam {
  double sigma_range{1.0};    ///< Standard deviation of the range error.
  double sigma_bearing{1.0};  ///< Standard deviation of the bearing error.
};

/// Generic landmark model for discrete detection sensors (both 2D and 3D).
/**
 * This class satisfies \ref SensorModelPage.
 *
 * This sensor model is a generalization of the model described in Probabilistic
 * Robotics \cite thrun2005probabilistic Chapter 6.6 .
 *
 * \tparam LandmarkMap class managing the list of known landmarks.
 * \tparam StateType type of the state of the particle.
 */
template <class LandmarkMap, class StateType>
class LandmarkSensorModel {
 public:
  /// State type of a particle.
  using state_type = StateType;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor, detection position in robot frame
  using measurement_type = std::vector<LandmarkPositionDetection>;
  /// Map representation type.
  using map_type = LandmarkMap;
  /// Parameter type that the constructor uses to configure the beam sensor model.
  using param_type = LandmarkModelParam;

  /// Constructs a LandmarkSensorModel instance.
  /**
   * \param params Parameters to configure this instance. See beluga::BeamModelParams for details.
   * \param landmark_map Map of landmarks to be used by the sensor model to compute importance weights
   *  for particle states.
   */
  explicit LandmarkSensorModel(param_type params, LandmarkMap landmark_map)
      : params_{std::move(params)}, landmark_map_{std::move(landmark_map)} {}

  /// Returns a state weighting function conditioned on landmark position detections.
  /**
   * \param detections 2D lidar hit points in the reference frame of filter particles.
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

      const auto detection_weight = [this, &robot_pose_in_world](const auto& detection) {
        // calculate range and detection_bearing_in_robot to the detection from the robot
        // the detection is already in robot frame
        const auto& detection_category = detection.category;
        const auto& detection_position_in_robot = detection.detection_position_in_robot;

        const auto detection_range_in_robot = detection_position_in_robot.norm();
        const auto detection_bearing_in_robot = detection_position_in_robot.normalized();

        // convert the detection to the world frame to query the map
        const auto detection_position_in_world = robot_pose_in_world * detection_position_in_robot;

        // find the closest matching landmark in the world map
        const auto opt_landmark_position_in_world =
            landmark_map_.find_nearest_landmark(detection_position_in_world, detection_category);

        // if we did not find a matching landmark, return 0.0
        if (!opt_landmark_position_in_world) {
          return 0.0;
        }

        // convert landmark pose to world frame
        // ignore height, because we are modelling the detection in 2D
        const auto& landmark_position_in_world = *opt_landmark_position_in_world;
        const auto landmark_in_robot_position = robot_pose_in_world.inverse() * landmark_position_in_world;

        const auto landmark_range_in_robot = landmark_in_robot_position.norm();
        const auto landmark_bearing_in_robot = landmark_in_robot_position.normalized();

        // calculate the aperture angle between the detection and the landmark
        const auto cos_aperture = landmark_bearing_in_robot.dot(detection_bearing_in_robot);
        const auto sin_aperture = landmark_bearing_in_robot.cross(detection_bearing_in_robot).norm();
        const auto bearing_error = std::atan2(sin_aperture, cos_aperture);

        const auto range_error = detection_range_in_robot - landmark_range_in_robot;

        // apply the error model from Probabilistic Robotics
        const auto range_error_prob =
            std::exp(-range_error * range_error / (2. * params_.sigma_range * params_.sigma_range));
        const auto bearing_error_prob =
            std::exp(-bearing_error * bearing_error / (2. * params_.sigma_bearing * params_.sigma_bearing));

        // We'll assume the probability of identification error to be zero
        const auto prob = range_error_prob * bearing_error_prob;

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

/// Sensor model based on discrete landmarks for 2D state types.
template <class LandmarkMap>
using LandmarkSensorModel2d = LandmarkSensorModel<LandmarkMap, Sophus::SE2d>;

/// Sensor model based on discrete landmarks for 3D state types.
template <class LandmarkMap>
using LandmarkSensorModel3d = LandmarkSensorModel<LandmarkMap, Sophus::SE3d>;

}  // namespace beluga

#endif
