// Copyright 2022 Ekumen, Inc.
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

#ifndef BELUGA_SENSOR_HPP
#define BELUGA_SENSOR_HPP

#include <beluga/sensor/likelihood_field_model.hpp>

/**
 * \file
 * \brief Includes all beluga sensor models.
 */

/**
 * \page SensorModelPage beluga named requirements: SensorModel
 * Requirements for a sensor model to be used in a beluga `ParticleFilter`.
 *
 * \section SensorModelRequirements Requirements
 * A type `T` satisfies the `SensorModel` requirements if the following is satisfied:
 * - `T::state_type` is a valid type, representing a particle state.
 * - `T::weight_type` is an arithmetic type, representing a particle weight.
 * - `T::measurement_type` is a valid type, representing a sensor measurement.
 *
 * Given:
 * - An instance `p` of `T`.
 * - A possibly const instance `cp` of `T`.
 * - A possibly const instance of `T::measurement_type` `m`.
 * - A possibly const instance of `T::state_type` `s`.
 *
 * Then:
 * - `p.update_sensor(m)` will update the sensor model with `p`.
 *   This will not actually update the particle weights, but the update done here
 *   will be used in the nexts `importance_weight()` method calls.
 * - `cp.importance_weight(s)` returns a `T::weight_type` instance representing the weight of the particle.
 * - `cp.generate_random_state()` returns a `T::state_type` instance.
 */

namespace beluga {

struct LaserSensorModelInterface2d {
  virtual ~LaserSensorModelInterface2d() = default;
  virtual void update_sensor(std::vector<std::pair<double, double>>) = 0;
};

}  // namespace beluga

#endif
