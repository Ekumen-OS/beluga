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

#include <openvdb/openvdb.h>
#include <openvdb/tools/VolumeToSpheres.h>

#include <Eigen/Core>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

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
 * \tparam OpenVDB grid type.
 */
template <typename GridT, typename U>
class Likelihood3DFieldModel {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE3d;

  /// Weight type of the particle.
  using weight_type = double;

  /// Measurement type given by the interface.
  using measurement_type = U;

  /// Map representation type.
  using map_type = GridT;

  /// Parameter type that the constructor uses to configure the likelihood field model.
  using param_type = Likelihood3DFieldModelParam;

  /// Constructs a Likelihood3DFieldModel instance.
  /**
   * \param params Parameters to configure this instance.
   *  See beluga::Likelihood3DFieldModelParam for details.
   * \param grid Narrow band Level set grid representing the static map that the sensor model
   *  uses to compute a likelihood field for lidar hits and compute importance weights
   *  for particle states.
   *  Currently only supports OpenVDB Level sets.
   */
  explicit Likelihood3DFieldModel(const param_type& params, const map_type& grid)
      : params_{params},
        looker_{openvdb::tools::ClosestSurfacePoint<map_type>::create(grid)},
        squared_sigma_{params.sigma_hit * params.sigma_hit},
        amplitude_{params.z_hit / (params.sigma_hit * std::sqrt(2 * Sophus::Constants<double>::pi()))},
        offset_{params.z_random / params.max_laser_distance} {
    /// Pre-computed parameters
    assert(squared_sigma_ > 0.0);
    assert(amplitude_ > 0.0);
    // Remember this OpenVDB DualGridSmpler
  }

  /// Returns a state weighting function conditioned on 3D lidar hits.
  /**
   * \param measurement 3D lidar measurement containing the hit points and the transform to the origin.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   *  and borrowing a reference to this sensor model (and thus their lifetime are bound).
   */
  [[nodiscard]] auto operator()(measurement_type&& measurement) const {
    const size_t pointcloud_size = measurement.points().size();
    // Transform each point from the sensor frame to the origin frame
    return [this,
            points = std::move(measurement.origin() * measurement.points())](const state_type& state) -> weight_type {
      std::vector<float> nb_distances;
      std::vector<openvdb::Vec3R> vdb_points;
      vdb_points.resize(pointcloud_size);

      // Transform each point to every particle state
      std::transform(points.cbegin(), points.cend(), vdb_points.begin(), [this, state](const auto& point) {
        // OpenVDB grid already in world coordinates
        const Eigen::Vector3d point_in_state_frame = state * point;
        return openvdb::Vec3R{point_in_state_frame.x(), point_in_state_frame.y(), point_in_state_frame.z()};
      });

      // Extract the distance to the closest surface for each point
      looker_.search(vdb_points, nb_distances);

      // Calculates the probality based on the distance
      return std::transform_reduce(
          nb_distances.cbegin(), nb_distances.cend(), 1.0, std::plus{}, [this](const auto& distance) {
            return amplitude_ * std::exp(-(distance * distance) / squared_sigma_) + offset_;
          });
    };
  }

 private:
  param_type params_;
  typename openvdb::tools::ClosestSurfacePoint<map_type>::Ptr looker_;
  const double squared_sigma_;
  const double amplitude_;
  const double offset_;
};

}  // namespace beluga

#endif
