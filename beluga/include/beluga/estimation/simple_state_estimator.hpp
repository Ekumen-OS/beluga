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

#ifndef BELUGA_ESTIMATION_SIMPLE_STATE_ESTIMATOR_HPP
#define BELUGA_ESTIMATION_SIMPLE_STATE_ESTIMATOR_HPP

// standard library
#include <numeric>
#include <utility>

// external
#include <sophus/se2.hpp>
#include <sophus/types.hpp>

// project
#include <beluga/estimation/utils/range_statistics.hpp>

/**
 * \file
 * \brief Implementation of a simple state estimator.
 */

namespace beluga {

/// Primary template for a simple state estimator.
/**
 * This class implements the EstimationInterface interface
 * and satisfies \ref StateEstimatorPage.
 *
 * It's an estimator that calculates the state mean and covariance of the
 * particles in the set. All particles are considered to have equal weight.
 *
 * \tparam Mixin The mixin that implements the particle filter interface.
 * \tparam StateType The type of the state to be estimated.
 * \tparam CovarianceType The type of the covariance matrix of the state to be estimated.
 */
template <class Mixin, class StateType, class CovarianceType>
class SimpleStateEstimator : public Mixin {
 public:
  /// Constructs a SimpleStateEstimator instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit SimpleStateEstimator(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// \copydoc EstimationInterface2d::estimate()
  [[nodiscard]] std::pair<StateType, CovarianceType> estimate() const final {
    return beluga::estimate(this->self().states());
  }
};

/// An alias template for the simple state estimator in 2D.
template <class Mixin>
using SimpleStateEstimator2d = SimpleStateEstimator<Mixin, Sophus::SE2d, Eigen::Matrix3d>;

}  // namespace beluga

#endif
