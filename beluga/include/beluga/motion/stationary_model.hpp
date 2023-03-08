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

#ifndef BELUGA_MOTION_STATIONARY_MODEL_HPP
#define BELUGA_MOTION_STATIONARY_MODEL_HPP

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
 * This class satisfies the \ref MotionModelPage "MotionModel" requirements.
 *
 * \tparam Mixin The mixed-in type.
 */
template <class Mixin>
struct StationaryModel : public Mixin {
 public:
  /// Update type of the motion model.
  using update_type = Sophus::SE2d;
  /// State type of a particle.
  using state_type = Sophus::SE2d;

  /// Constructs a StationaryModel instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit StationaryModel(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Applies motion to a particle.
  /**
   * The updated state will be centered around `state` with some covariance.
   *
   * \param state The particle state to apply the motion to.
   * \return The updated paticle state.
   */
  template <class Generator>
  [[nodiscard]] state_type apply_motion(const state_type& state, Generator& gen) const {
    auto distribution = std::normal_distribution<>{0, 0.02};
    return state * Sophus::SE2d{Sophus::SO2d{distribution(gen)}, Eigen::Vector2d{distribution(gen), distribution(gen)}};
  }

  /// Updates motion model.
  /**
   * For the stationary model, updates are ignored.
   */
  void update_motion(const update_type&) final {}

  /// Recovers latest motion update.
  /**
   * For the stationary model, we don't ever have motion updates.
   */
  [[nodiscard]] std::optional<update_type> latest_motion_update() const { return std::nullopt; }
};

}  // namespace beluga

#endif
