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
#include <beluga/motion/omnidirectional_drive_model.hpp>
#include <beluga/motion/stationary_model.hpp>

/**
 * \file
 * \brief Includes all Beluga motion models.
 */

/**
 * \page MotionModelPage Beluga named requirements: MotionModel
 * Requirements for a motion model to be used in a Beluga `ParticleFilter`.
 *
 * \section MotionModelRequirements Requirements
 * A type `T` satisfies the `MotionModel` requirements if the following is satisfied:
 * - `T::state_type` is a valid type, representing a particle state.
 * - `T::update_type` is a valid type, representing a motion model update.
 *
 * Given:
 * - An instance `p` of `T`.
 * - A possibly const instance `cp` of `T`.
 * - A possibly const instance of `T::update_type` `u`.
 * - A possibly const instance of `T::state_type` `s`.
 *
 * Then:
 * - `p.update_motion(u)` will update the motion model with `u`.
 *   This will not actually update the particle states, but the update done here
 *   will be used in subsequent calls to the `apply_motion()` method.
 * - `cp.apply_motion(s)` returns a `T::state_type`, that is the result of applying the motion model
 *   to `s` based on the updates.
 *
 * \section MotionModelLinks See also
 * - beluga::DifferentialDriveModel
 * - beluga::OmnidirectionalDriveModel
 * - beluga::StationaryModel
 */

namespace beluga {

/// Pure abstract class representing the odometry motion model interface.
struct OdometryMotionModelInterface2d {
  /// Update type of the motion model.
  using update_type = Sophus::SE2d;

  /// Virtual destructor.
  virtual ~OdometryMotionModelInterface2d() = default;

  /// Updates the motion model with the latest odometry data.
  /**
   * This method updates the motion model with the information
   * it needs to apply the motion to each particle.
   * The motion is applied by subsequent calls to the `apply_motion()`
   * method provided by the same mixin component.
   *
   * \param pose Latest odometry update.
   */
  virtual void update_motion(const update_type& pose) = 0;
};

}  // namespace beluga

#endif
