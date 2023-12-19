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

#ifndef BELUGA_SENSOR_HPP
#define BELUGA_SENSOR_HPP

/**
 * \file
 * \brief Includes all Beluga sensor models and their interfaces.
 */

/**
 * \page SensorModelPage Beluga named requirements: SensorModel
 * Requirements for a sensor model to be used in a Beluga `ParticleFilter`.
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
 * - A possibly const instance `m` of `T::measurement_type`.
 * - A possibly const instance `s` of `T::state_type`.
 *
 * Then:
 * - `p.update_sensor(m)` will update the sensor model with `p`.
 *   This will not actually update the particle weights, but the update done here
 *   will be used in the following `importance_weight()` method calls.
 * - `cp.importance_weight(s)` returns a `T::weight_type` instance representing the weight of the particle.
 * - `cp.make_random_state()` returns a `T::state_type` instance.
 *
 * \section SensorModelLinks See also
 * - beluga::LikelihoodFieldModel
 * - beluga::BeamSensorModel
 * - beluga::LandmarkSensorModel2d
 * - beluga::BearingSensorModel2d
 */

// interfaces
#include <beluga/interfaces/bearing_sensor_model_interface.hpp>
#include <beluga/interfaces/landmark_sensor_model_interface.hpp>
#include <beluga/interfaces/laser_sensor_model_interface_2d.hpp>

// implementations
#include <beluga/sensor/beam_model.hpp>
#include <beluga/sensor/bearing_sensor_model.hpp>
#include <beluga/sensor/landmark_sensor_model.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>

#endif
