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
 * @brief Canonical state schemas shared across operations.
 *
 * Operations and constraints interoperate by agreeing on a schema: a set of
 * variable names with fixed semantics and dimensions. The names below define
 * the compact schema currently supported by the input probes and the kinematic
 * state connector.
 *
 * The compact schema stores the state as a whole pose plus fused motion
 * quantities. Coordinates are listed in body-frame tangent order:
 * - 2D: `pose` is SE(2); `velocity` is R³ `[vx, vy, omega]`; `acceleration`
 *   is R³ `[ax, ay, alpha]`.
 * - 3D: `pose` is SE(3); `velocity` is R⁶ `[vx, vy, vz, wx, wy, wz]`;
 *   `acceleration` is R⁶ `[ax, ay, az, alpha_x, alpha_y, alpha_z]`.
 *
 * In 2D, `omega` and `alpha` are the z components of angular velocity and
 * angular acceleration respectively (`wz` and `alpha_z` in the 3D notation).
 * Velocity and acceleration coordinates follow the body-frame tangent ordering
 * used by the manifold's exponential map, so `SE2::exp(velocity * dt)`
 * propagates a pose without reordering. Accelerometers observe only the linear
 * part of `acceleration`.
 *
 * The granular schema (separate position, orientation, linear velocity and
 * angular velocity variables) is a future extension and is not yet implemented.
 */

#include <string_view>

namespace beluga_estimation {

/// @brief Name of the pose variable in the compact schema.
inline constexpr std::string_view kPoseVariableName = "pose";

/// @brief Name of the fused velocity variable in the compact schema.
inline constexpr std::string_view kVelocityVariableName = "velocity";

/// @brief Name of the fused acceleration variable in the compact schema.
inline constexpr std::string_view kAccelerationVariableName = "acceleration";

}  // namespace beluga_estimation
