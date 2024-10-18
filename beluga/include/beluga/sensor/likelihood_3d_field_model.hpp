// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_SENSOR_LIKELIHOOD_3D_FIELD_MODEL_HPP
#define BELUGA_SENSOR_LIKELIHOOD_3D_FIELD_MODEL_HPP

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <beluga/algorithm/distance_map.hpp>
#include <beluga/sensor/data/occupancy_grid.hpp>
#include <beluga/sensor/data/value_grid.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

/**
 * \file
 * \brief Implementation of a likelihood field sensor model for 3D Lidars.
 */

namespace beluga {

/// Parameters used to construct a Likelihood3DFieldModel instance.
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.4, particularly Table 6.3.
 */
struct Likelihood3DFieldModelParam {
  /// Voxel size in meters.
  double voxel_size = 0.07;
  /// Maximum distance to obstacle.
  /**
   * When creating a distance map, if the distance to an obstacle is higher than the value specified here,
   * then this value will be used.
   */
  double max_obstacle_distance = 100.0;
  /// Maximum range of a laser ray.
  double max_laser_distance = 2.0;
  /// Weight used to combine the probability of hitting an obstacle.
  double z_hit = 0.5;
  /// Weight used to combine the probability of random noise in perception.
  double z_random = 0.5;
  /// Standard deviation of a gaussian centered arounds obstacles.
  /**
   * Used to calculate the probability of the obstacle being hit.
   */
  double sigma_hit = 0.2;
};

/// Likelihood field sensor model for range finders.
/**
 * This model relies on a pre-computed likelihood map of the environment.
 * It is less computationally intensive than the beluga::BeamSensorModel
 * because no ray-tracing is required, and it can also provide better
 * performance in environments with non-smooth occupation maps. See
 * Probabilistic Robotics \cite thrun2005probabilistic, Chapter 6.4,
 * for further reference.
 *
 * \note This class satisfies \ref SensorModelPage.
 *
 * \tparam OccupancyGrid Type representing an occupancy grid.
 *  It must satisfy \ref OccupancyGrid2Page.
 */
template <typename T, class U>
class Likelihood3DFieldModel {
 public:
  /// State type of a particle.
  // planar movement but add Z
  using state_type = Sophus::SE3d;

  /// Weight type of the particle.
  using weight_type = double;

  /// Measurement type of the sensor: a point cloud for the XXXXXXXXXXX.
  using measurement_type = U;

  /// Map representation type.
  // openvdb, algo asi
  using map_type = T;

  /// Parameter type that the constructor uses to configure the likelihood field model.
  // nombre? Likelihood3DFieldModel?? el modelo es el mismo solo que los ptos son 3d
  // pero el modelo sigue siendo 2D
  using param_type = Likelihood3DFieldModel;

  /// Constructs a Likelihood3DFieldModel instance.
  /**
   * \param params Parameters to configure this instance.
   *  See beluga::Likelihood3DFieldModelParam for details.
   * \param grid Occupancy grid representing the static map that the sensor model
   *  uses to compute a likelihood field for lidar hits and compute importance weights
   *  for particle states.
   */
  explicit Likelihood3DFieldModel(const param_type& params, const map_type& map)
      : params_{params}, likelihood_field_map_{std::move(map)} {
    // encontrar el punto mas cercano usnado
    // probar esto sacarlo dela funcion
    // DualGridSmpler
    looker.create(likelihood_field_map_);
  }

  /// Returns the likelihood field, constructed from the provided map.
  [[nodiscard]] const auto& likelihood_field() const { return likelihood_field_map_; }

  /// Returns a state weighting function conditioned on 2D lidar hits.
  /**
   * \param points 2D lidar hit points in the reference frame of particle states.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   *  and borrowing a reference to this sensor model (and thus their lifetime are bound).
   */
  [[nodiscard]] auto operator()(measurement_type&& points) const {
    return [this, points = std::move(points)](const state_type& state) -> weight_type {
      // map already in world coordinates
      const auto unknown_space_occupancy_prob = static_cast<float>(1. / params_.max_laser_distance);

      return std::transform_reduce(
          points.cbegin(), points.cend(), 1.0, std::plus{},
          [this, state, unknown_space_occupancy_prob](const auto& point) {
            const auto result = state * point;

            std::vector<float> distances;
            std::vector<Vec3R> points;
            // output list of closest surface point distances
            // looker.search(points, distances);

            return 0.1;
          });
    };
  }

  /// Update the sensor model with a new occupancy grid map.
  /**
   * This method re-computes the underlying likelihood field.
   *
   * \param grid New occupancy grid representing the static map.
   */
  void update_map(const map_type& map) { likelihood_field_map_{std::move(map)}; }

 private:
  param_type params_;
  T likelihood_field_map_;
  Sophus::SE3d world_to_likelihood_field_map_transform_;
  Openvdb::ClosestSurfacePoint<map_type> looker;
};

}  // namespace beluga

#endif
