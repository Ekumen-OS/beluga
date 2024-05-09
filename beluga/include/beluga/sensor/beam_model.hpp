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

#ifndef BELUGA_SENSOR_BEAM_MODEL_HPP
#define BELUGA_SENSOR_BEAM_MODEL_HPP

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <beluga/algorithm/raycasting.hpp>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

/**
 * \file
 * \brief Implementation of a beam sensor model for range finders.
 */

namespace beluga {

/// Parameters used to construct a BeamSensorModel instance.
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic table 6.2.
 */
struct BeamModelParam {
  /// Weight associated with good but noisy readings.
  double z_hit{0.5};
  /// Weight associated with unexpected obstacles.
  double z_short{0.5};
  /// Weight associated with max range readings.
  double z_max{0.05};
  /// Weight associated with random readings.
  double z_rand{0.05};
  /// Standard deviation of the gaussian noise associated with hits.
  double sigma_hit{0.2};
  /// Intrinsic parameter assoaciated with short readings distribution.
  double lambda_short{0.1};
  /// Maximum beam range. This is the expected value in case of a miss.
  double beam_max_range{60};
};

/// Beam sensor model for range finders.
/**
 * This model closely matches the physical behavior of a lidar, but it is
 * computationally expensive because a ray-tracing operation needs to be
 * performed for each particle and individual range reading. Also, this
 * model can experience degraded performance in cluttered environments
 * due the inherent lack of smoothness of the sensor readings in such
 * settings. See Probabilistic Robotics \cite thrun2005probabilistic
 * Chapter 6.2 for further reference.
 *
 * \note This class satisfies \ref SensorModelPage.
 *
 * \tparam OccupancyGrid Type representing an occupancy grid.
 *  It must satisfy \ref OccupancyGrid2Page.
 */
template <class OccupancyGrid>
class BeamSensorModel {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<std::pair<double, double>>;
  /// Map representation type.
  using map_type = OccupancyGrid;
  /// Parameter type that the constructor uses to configure the beam sensor model.
  using param_type = BeamModelParam;

  /// Constructs a BeamSensorModel instance.
  /**
   * \param params Parameters to configure this instance.
   *  See beluga::BeamModelParams for details.
   * \param grid Occupancy grid representing the static map that the sensor model
   *  uses to raytrace lidar beams and compute importance weights for particle states.
   */
  explicit BeamSensorModel(const param_type& params, OccupancyGrid grid) : params_{params}, grid_{std::move(grid)} {}

  /// Returns a state weighting function conditioned on 2D lidar hits.
  /**
   * \param points 2D lidar hit points in the reference frame of particle states.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   *  and borrowing a reference to this sensor model (and thus their lifetime are bound).
   */
  [[nodiscard]] auto operator()(measurement_type&& points) const {
    return [this, points = std::move(points)](const state_type& state) -> weight_type {
      const auto beam = Ray2d{grid_, state, params_.beam_max_range};
      const double n = 1. / (std::sqrt(2. * M_PI) * params_.sigma_hit);
      return std::transform_reduce(
          points.cbegin(), points.cend(), 0.0, std::plus{}, [this, &beam, n](const auto& point) {
            // TODO(Ramiro): We're converting from range + bearing to cartesian points in the ROS node, but we want
            // range
            // + bearing here. We might want to make that conversion in the likelihood model instead, and let the
            // measurement type be range, bearing instead of x, y.

            // Compute the range according to the measurement.
            const double z = std::sqrt(point.first * point.first + point.second * point.second);

            // dirty hack to prevent SO2d from calculating the hypot again to normalize the vector.
            auto beam_bearing = Sophus::SO2d{};
            beam_bearing.data()[0] = point.first / z;
            beam_bearing.data()[1] = point.second / z;

            // Compute range according to the map (raycasting).
            const double z_mean = beam.cast(beam_bearing).value_or(params_.beam_max_range);
            // 1: Correct range with local measurement noise.
            const double eta_hit =
                2. / (std::erf((params_.beam_max_range - z_mean) / (std::sqrt(2.) * params_.sigma_hit)) -
                      std::erf(-z_mean / (std::sqrt(2.) * params_.sigma_hit)));
            const double d = (z - z_mean) / params_.sigma_hit;
            double pz = params_.z_hit * eta_hit * n * std::exp(-(d * d) / 2.);

            // 2: Unexpected objects.
            if (z < z_mean) {
              const double eta_short = 1. / (1. - std::exp(-params_.lambda_short * z_mean));
              pz += params_.z_short * params_.lambda_short * eta_short * std::exp(-params_.lambda_short * z);
            }

            // 3 and 4: Max range return or random return.
            if (z < params_.beam_max_range) {
              pz += params_.z_rand / params_.beam_max_range;
            } else {
              pz += params_.z_max;
            }

            // TODO(glpuga): Investigate why AMCL and QuickMCL both use this formula for the weight.
            // See https://github.com/Ekumen-OS/beluga/issues/153
            return pz * pz * pz;
          });
    };
  }

  /// Update the sensor model with a new occupancy grid map.
  /**
   * \param map New occupancy grid representing the static map.
   */
  void update_map(map_type&& map) { grid_ = std::move(map); }

 private:
  param_type params_;
  OccupancyGrid grid_;
};

}  // namespace beluga

#endif
