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

#ifndef BELUGA_ESTIMATION_HPP
#define BELUGA_ESTIMATION_HPP

/**
 * \file
 * \brief Implementation of algorithms that allow calculating the estimated state of
 *  a particle filter.
 */

/**
 * \page StateEstimatorPage Beluga named requirements: StateEstimator
 * The requirements that a state estimator must satisfy.
 *
 * \section StateEstimationRequirements Requirements
 * `T` is a `StateEstimator` if given a (possibly const) instance `p` of `T`, the following is satisfied:
 * - `p.estimate()` is a valid expression.
 * - `std::get<0>(p.estimate())` is valid.
 *   `decltype(std::get<0>(p.estimate()))` represents the type of the estimated state.
 * - `std::get<1>(p.estimate())` is valid.
 *   `decltype(std::get<1>(p.estimate()))` represents the type of the dispersion of the estimation.
 *
 * \section StateEstimationLinks See also
 * - beluga::SimpleStateEstimator<Mixin, Sophus::SE2d>
 * - beluga::WeightedStateEstimator<Mixin, Sophus::SE2d>
 */

// interfaces
#include <beluga/interfaces/estimation_interface.hpp>

// implementations
#include <beluga/estimation/cluster_based_estimator.hpp>
#include <beluga/estimation/simple_state_estimator.hpp>
#include <beluga/estimation/weighted_state_estimator.hpp>

#endif
