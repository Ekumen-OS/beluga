// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_AMCL__PARTICLE_FILTERING_HPP_
#define BELUGA_AMCL__PARTICLE_FILTERING_HPP_

#include <string_view>
#include <variant>

#include <beluga/localization.hpp>

#include <beluga/mixin.hpp>
#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>

#include <beluga_amcl/occupancy_grid.hpp>

#include <sophus/se2.hpp>

namespace beluga_amcl
{

namespace
{

constexpr std::string_view kDifferentialModelName = "differential_drive";
constexpr std::string_view kOmnidirectionalModelName = "omnidirectional_drive";
constexpr std::string_view kStationaryModelName = "stationary";

constexpr std::string_view kNav2DifferentialModelName = "nav2_amcl::DifferentialMotionModel";
constexpr std::string_view kNav2OmnidirectionalModelName = "nav2_amcl::OmniMotionModel";

constexpr std::string_view kLikelihoodFieldModelName = "likelihood_field";
constexpr std::string_view kBeamSensorModelName = "beam";

}  // namespace

using LaserLocalizationInterface2d =
  beluga::LaserLocalizationInterface2d<OccupancyGrid>;

using Stationary = beluga::mixin::descriptor<beluga::StationaryModel>;
using DifferentialDrive = beluga::mixin::descriptor<
  beluga::DifferentialDriveModel,
  beluga::DifferentialDriveModelParam>;
using OmnidirectionalDrive = beluga::mixin::descriptor<
  beluga::OmnidirectionalDriveModel,
  beluga::OmnidirectionalDriveModelParam>;

using MotionDescriptor = std::variant<Stationary, DifferentialDrive, OmnidirectionalDrive>;

using LikelihoodField = beluga::mixin::descriptor<
  ciabatta::curry<beluga::LikelihoodFieldModel, OccupancyGrid>::mixin,
  beluga::LikelihoodFieldModelParam>;

using BeamSensorModel = beluga::mixin::descriptor<
  ciabatta::curry<beluga::BeamSensorModel, OccupancyGrid>::mixin,
  beluga::BeamModelParam>;

using SensorDescriptor = std::variant<LikelihoodField, BeamSensorModel>;

template<class MotionDescriptor, class SensorDescriptor>
using MonteCarloLocalization2d =
  beluga::MonteCarloLocalization2d<MotionDescriptor, SensorDescriptor, beluga_amcl::OccupancyGrid>;

template<class MotionDescriptor, class SensorDescriptor>
using AdaptiveMonteCarloLocalization2d =
  beluga::AdaptiveMonteCarloLocalization2d<MotionDescriptor, SensorDescriptor, OccupancyGrid>;

/// Initializes particle filter states given pose `mean` and `covariance`.
/**
 * \param mean Particle distribution pose mean.
 * \param covariance Particle distribution pose covariance.
 * \param particle_filter Particle filter to be initialized.
 */
void initialize_with_pose(
  const Sophus::SE2d & mean,
  const Eigen::Matrix3d & covariance,
  LaserLocalizationInterface2d * particle_filter);

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__PARTICLE_FILTERING_HPP_
