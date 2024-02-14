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
#include <tuple>
#include <type_traits>

#include <beluga/type_traits/tuple_traits.hpp>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

/**
 * \file
 * \brief Implementation of a stationary motion model.
 */

namespace beluga {

/// A stationary motion model.
/**
 * This class satisfies \ref MotionModelPage.
 *
 * It ignores all odometry updates and only adds Gaussian noise to the input states.
 */
class StationaryModel {
 public:
  /// Current and previous odometry estimates as motion model control action.
  using control_type = std::tuple<Sophus::SE2d, Sophus::SE2d>;
  /// 2D pose as motion model state (to match that of the particles).
  using state_type = Sophus::SE2d;

  /// Computes a state sampling function conditioned on a given control action.
  /**
   * The updated state will be centered around `state` with some covariance.
   * For this model, control actions are ignored.
   *
   * \tparam Control A tuple-like container matching the model's `control_action_type`.
   * \return a callable satisfying \ref StateSamplingFunctionPage.
   */
  template <class Control, typename = common_tuple_type_t<Control, control_type>>
  [[nodiscard]] auto operator()([[maybe_unused]] Control&&) const {
    return [](const state_type& state, auto& gen) {
      static thread_local auto distribution = std::normal_distribution<>{0, 0.02};
      return state *
             Sophus::SE2d{Sophus::SO2d{distribution(gen)}, Eigen::Vector2d{distribution(gen), distribution(gen)}};
    };
  }
};

}  // namespace beluga

#endif
