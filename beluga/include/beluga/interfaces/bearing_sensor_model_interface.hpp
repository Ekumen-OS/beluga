// Copyright 2022-2023 Ekumen, Inc.
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

#ifndef BELUGA_INTERFACES_BEARING_SENSOR_MODEL_INTERFACE_HPP
#define BELUGA_INTERFACES_BEARING_SENSOR_MODEL_INTERFACE_HPP

// standard library
#include <cstdint>
#include <vector>

// project
#include <beluga/types/landmark_detection_types.hpp>

/**
 * \file
 * \brief Includes the interface for bearing sensor models.
 */

namespace beluga {

/// Pure abstract class representing the bearing sensor model interface.
/**
 * \tparam Map Bearing landmarks representation type.
 */
template <class Map>
struct BearingSensorModelInterface {
  /// Measurement type of the sensor: vector of landmark detections
  using measurement_type = std::vector<LandmarkBearingDetection>;

  /// Virtual destructor.
  virtual ~BearingSensorModelInterface() = default;

  /// Update the sensor model with the measured points.
  /**
   * This method updates the sensor model with the information
   * it needs to compute the weight of each particle.
   * The weight of each particle is calculated by subsequent calls to
   * the `importance_weight()` method provided by the same mixin
   * component.
   *
   * \param points The discrete sensor detections.
   */
  virtual void update_sensor(measurement_type&& points) = 0;

  /// Update the sensor model with a new map.
  /**
   * This method updates the sensor model with a new map,
   * i.e. a representation of the environment, that it needs
   * to compute the weight of each particle.
   *
   * \param map The range finder map.
   */
  virtual void update_map(Map&& map) = 0;
};

}  // namespace beluga

#endif
