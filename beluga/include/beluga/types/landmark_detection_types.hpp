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

#ifndef BELUGA_TYPES_LANDMARK_DETECTION_TYPES_HPP
#define BELUGA_TYPES_LANDMARK_DETECTION_TYPES_HPP

// external
#include <eigen3/Eigen/Core>

// standard library
#include <cstdint>

/**
 * \file
 * \brief Auxiliar types for landmark models.
 */

namespace beluga {

using LandmarkCategory = uint32_t;          ///< Type used to represent landmark categories.
using LandmarkPosition3 = Eigen::Vector3d;  ///< Position of a landmark in the world reference frame.
using LandmarkBearing3 = Eigen::Vector3d;   ///< Bearing of a landmark in the sensor reference frame.

/// Landmark bearing detection data
struct LandmarkMapBoundaries {
  double x_min;  ///< Minimum x coordinate of the map.
  double x_max;  ///< Maximum x coordinate of the map.
  double y_min;  ///< Minimum y coordinate of the map.
  double y_max;  ///< Maximum y coordinate of the map.
  double z_min;  ///< Minimum z coordinate of the map.
  double z_max;  ///< Maximum z coordinate of the map.
};

/// Landmark bearing detection data
struct LandmarkPositionDetection {
  LandmarkPosition3 detection_position_in_robot;  ///< Detection pose in the robot reference frame.
  LandmarkCategory category;                      ///< Category of the landmark detection (i.e. landmark type).
};

/// Landmark bearing detection data
struct LandmarkBearingDetection {
  LandmarkBearing3 detection_bearing_in_sensor;  ///< Bearing vector of the detection in the sensor reference frame.
  LandmarkCategory category;                     ///< Category of the landmark detection (i.e. landmark type).
};

}  // namespace beluga

#endif
