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

#ifndef BELUGA_MOTION_HPP
#define BELUGA_MOTION_HPP

#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/motion/differential_velocity_drive_model.hpp>
#include <beluga/motion/omnidirectional_drive_model.hpp>
#include <beluga/motion/stationary_model.hpp>

/**
 * \file
 * \brief Includes all Beluga motion models.
 *
 * Motion models in Beluga include:
 * - Position-based models: Use pose differences (DifferentialDriveModel, OmnidirectionalDriveModel, StationaryModel)
 * - Velocity-based models: Use timestamped poses to calculate velocities (DifferentialVelocityDriveModel)
 */

/**
 * \page MotionModelPage Beluga named requirements: MotionModel
 * Requirements for a motion model to be used in a Beluga `ParticleFilter`.
 *
 * \section MotionModelRequirements Requirements
 * A type `T` satisfies the `MotionModel` requirements if the following is satisfied:
 * - `T::state_type` is a valid type, representing a particle state.
 * - `T::control_type` is a valid type, representing a control action
 *    to condition the motion model.
 *
 * Given:
 * - A possibly const instance `cp` of `T`.
 * - A possibly const instance `u` of `T::control_type` (or convertible type `U`).
 *
 * Then:
 * - `cp(u)` returns a callable satisfying \ref StateSamplingFunctionPage for `T::state_type`.
 *
 * \section StateSamplingFunctionPage Beluga named requirements: StateSamplingFunction
 * Requirements for a callable to sample a motion distribution when propagating
 * particle states in a Beluga `ParticleFilter`.
 *
 * \section StateSamplingFunctionRequirements Requirements
 * A type `F` satisfies the `StateSamplingFunction` requirements for some state type `S` if:
 *
 * Given:
 * - A possibly const instance `fn` of `F`.
 * - A possible const instance `s` of `S`.
 * - An instance `e` satisfying [URNG](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
 * requirements.
 *
 * Then:
 * - `fn(s, e)` returns another sampled state `ss` of `S`.
 *
 * \section MotionModelLinks See also
 * - beluga::DifferentialDriveModel
 * - beluga::OmnidirectionalDriveModel
 * - beluga::StationaryModel
 * - beluga::DifferentialVelocityDriveModel
 */

#endif
