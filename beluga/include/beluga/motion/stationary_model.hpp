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

#ifndef BELUGA_MOTION_STATIONARY_MODEL_HPP
#define BELUGA_MOTION_STATIONARY_MODEL_HPP

#include <optional>
#include <random>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

/**
 * \file
 * \brief Implementation of a stationary motion model.
 */

namespace beluga {

/// A stationary motion model.
/**
 * This class implements the OdometryMotionModelInterface2d and satisfies \ref MotionModelPage.
 *
 * It ignores all odometry updates and only adds Gaussian noise to the
 * input states.
 *
 * \tparam Mixin The mixed-in type with no particular requirements.
 */
template <class Mixin>
class StationaryModel : public Mixin {
 public:
  /// Update type of the motion model.
  using update_type = Sophus::SE2d;
  /// State type of a particle.
  using state_type = Sophus::SE2d;

  /// Constructs a StationaryModel instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit StationaryModel(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Applies motion to the given particle state.
  /**
   * The updated state will be centered around `state` with some covariance.
   *
   * \tparam Generator  A random number generator that must satisfy the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param state The state of the particle to which the motion will be applied.
   * \param gen An uniform random bit generator object.
   * \return The new particle state.
   */
  template <class Generator>
  [[nodiscard]] state_type apply_motion(const state_type& state, Generator& gen) const {
    auto distribution = std::normal_distribution<>{0, 0.02};
    return state * Sophus::SE2d{Sophus::SO2d{distribution(gen)}, Eigen::Vector2d{distribution(gen), distribution(gen)}};
  }

  /**
   * \copydoc OdometryMotionModelInterface2d::update_motion(const Sophus::SE2d&)
   *
   * For this model, odometry updates are ignored.
   */
  void update_motion([[maybe_unused]] const update_type& pose) final {}
};

}  // namespace beluga

#endif
