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

#ifndef BELUGA_SENSOR_LIKELIHOOD_FIELD_MODEL_HPP
#define BELUGA_SENSOR_LIKELIHOOD_FIELD_MODEL_HPP

#include <algorithm>
#include <beluga/sensor/likelihood_field_model_base.hpp>
#include <cmath>
#include <random>
#include <vector>

/**
 * \file
 * \brief Implementation of a likelihood field sensor model for range finders.
 */

namespace beluga {

/// Parameters used to construct a LikelihoodFieldModel instance.
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.4, particularly Table 6.3.
 */
using LikelihoodFieldModelParam = LikelihoodFieldModelBaseParam;

/// Likelihood field sensor model for range finders.
/**
 * @copydoc LikelihoodFieldModelBase
 */
template <class OccupancyGrid>
class LikelihoodFieldModel : public LikelihoodFieldModelBase<OccupancyGrid> {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<std::pair<double, double>>;
  /// Map representation type.
  using map_type = OccupancyGrid;
  /// Parameter type that the constructor uses to configure the likelihood field model.
  using param_type = LikelihoodFieldModelParam;

  /// Constructs a LikelihoodFieldModel instance.
  /**
   * @copydoc LikelihoodFieldModelBase::LikelihoodFieldModelBase
   */
  explicit LikelihoodFieldModel(const param_type& params, const map_type& grid)
      : LikelihoodFieldModelBase<OccupancyGrid>(params, grid) {}

  /// Returns a state weighting function conditioned on 2D lidar hits.
  /**
   * \param points 2D lidar hit points in the reference frame of particle states.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   *  and borrowing a reference to this sensor model (and thus their lifetime are bound).
   */
  [[nodiscard]] auto operator()(measurement_type&& points) const {
    return [this, points = std::move(points)](const state_type& state) -> weight_type {
      const auto transform = this->world_to_likelihood_field_transform_ * state;
      const auto x_offset = transform.translation().x();
      const auto y_offset = transform.translation().y();
      const auto cos_theta = transform.so2().unit_complex().x();
      const auto sin_theta = transform.so2().unit_complex().y();
      const auto unknown_space_occupancy_prob = static_cast<float>(1. / this->params_.max_laser_distance);
      return std::transform_reduce(
          points.cbegin(), points.cend(), 1.0, std::plus{},
          [this, x_offset, y_offset, cos_theta, sin_theta, unknown_space_occupancy_prob](const auto& point) {
            // Transform the end point of the laser to the grid local coordinate system.
            // Not using Eigen/Sophus because they make the routine x10 slower.
            // See `benchmark_likelihood_field_model.cpp` for reference.
            const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
            const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
            const auto pz =
                static_cast<double>(this->likelihood_field_.data_near(x, y).value_or(unknown_space_occupancy_prob));
            // TODO(glpuga): Investigate why AMCL and QuickMCL both use this formula for the weight.
            // See https://github.com/Ekumen-OS/beluga/issues/153
            return pz * pz * pz;
          });
    };
  }
};

}  // namespace beluga

#endif
