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
 * - A possibly const instance `cp` of `T`.
 * - A movable instance `m` of `T::measurement_type`.
 *
 * Then:
 * - `cp(m)` returns a callable satisfying \ref StateWeightingFunctionPage
 * for states of `T::state_type` and weights of `T::weight_type` types.
 *
 * \section StateWeightingFunctionPage Beluga named requirements: StateWeightingFunction
 * Requirements on a callable used for reweighting particle states in a Beluga `ParticleFilter`.
 *
 * \section StateWeightingFunctionRequirements Requirements
 * A type `F` satisfies the `StateWeightingFunction` requirements for some state type `S`
 * and weight type `W` if:
 *
 * Given:
 * - A possibly const instance `fn` of `F`.
 * - A possible const instance `s` of `S`.
 *
 * Then:
 * - `fn(s)` returns a weight `w` of `W`.
 *
 * \section SensorModelLinks See also
 * - beluga::LikelihoodFieldModel
 * - beluga::LikelihoodFieldModel3
 * - beluga::BeamSensorModel
 * - beluga::LandmarkSensorModel
 * - beluga::BearingSensorModel
 */

#include <beluga/sensor/beam_model.hpp>
#include <beluga/sensor/bearing_sensor_model.hpp>
#include <beluga/sensor/landmark_sensor_model.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>
#include <beluga/sensor/ndt_sensor_model.hpp>

#endif
