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

#ifndef BELUGA_INTERFACES_ESTIMATION_INTERFACE_HPP
#define BELUGA_INTERFACES_ESTIMATION_INTERFACE_HPP

// standard library
#include <tuple>

// external
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/types.hpp>

/**
 * \file
 * \brief Interface for state estimation mixins.
 */

namespace beluga {

/**
 * \brief Interface for state estimation mixins.
 *
 * \tparam StateType The type of the state to be estimated.
 * \tparam CovarianceType The type of the covariance matrix of the state to be estimated.
 */
template <typename StateType, typename CovarianceType>
struct EstimationInterface {
  /// Virtual destructor.
  virtual ~EstimationInterface() = default;

  /// Returns the estimate state of the particle filter.
  /**
   * \return The estimated pose and its covariance matrix.
   */
  [[nodiscard]] virtual std::pair<StateType, CovarianceType> estimate() const = 0;
};

/// @brief Pure abstract class representing the estimation interface for 2D filters
using EstimationInterface2d = EstimationInterface<Sophus::SE2d, Sophus::Matrix3d>;

/// @brief Pure abstract class representing the estimation interface for 3D filters
using EstimationInterface3d = EstimationInterface<Sophus::SE3d, Sophus::Matrix6d>;

}  // namespace beluga

#endif
