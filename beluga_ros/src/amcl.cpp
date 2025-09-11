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

#include <beluga_ros/amcl.hpp>

#include <beluga/actions/assign.hpp>
#include <beluga/actions/normalize.hpp>
#include <beluga/actions/propagate.hpp>
#include <beluga/actions/reweight.hpp>
#include <beluga/algorithm/cluster_based_estimation.hpp>
#include <beluga/views/random_intersperse.hpp>
#include <beluga/views/take_while_kld.hpp>
#include <cmath>

namespace beluga_ros {

Amcl::Amcl(
    beluga_ros::OccupancyGrid map,
    motion_model_variant motion_model,
    sensor_model_variant sensor_model,
    const AmclParams& params = AmclParams(),
    execution_policy_variant execution_policy = std::execution::seq)
    : params_{params},
      map_distribution_{map},
      motion_model_{std::move(motion_model)},
      sensor_model_{std::move(sensor_model)},
      execution_policy_{std::move(execution_policy)},
      spatial_hasher_{params_.spatial_resolution_x, params_.spatial_resolution_y, params_.spatial_resolution_theta},
      random_probability_estimator_{params_.alpha_slow, params_.alpha_fast},
      update_policy_{beluga::policies::on_motion<Sophus::SE2d>(params_.update_min_d, params_.update_min_a)},
      resample_policy_{beluga::policies::every_n(params_.resample_interval)} {
  if (params_.selective_resampling) {
    resample_policy_ = resample_policy_ && beluga::policies::on_effective_size_drop;
  }
}

void Amcl::update_map(beluga_ros::OccupancyGrid map) {
  map_distribution_ = beluga::MultivariateUniformDistribution{map};
  std::visit([&](auto& sensor_model) { sensor_model.update_map(std::move(map)); }, sensor_model_);
}

// Overloaded update method for LaserScan.
auto Amcl::update(Sophus::SE2d base_pose_in_odom, beluga_ros::LaserScan laser_scan)
    -> std::optional<std::pair<Sophus::SE2d, Sophus::Matrix3d>> {
  // TODO(nahuel): Remove this once we update the measurement type.
  auto measurement = laser_scan.points_in_cartesian_coordinates() |  //
                     ranges::views::transform([&laser_scan](const auto& p) {
                       const auto result = laser_scan.origin() * Sophus::Vector3d{p.x(), p.y(), 0};
                       return std::make_pair(result.x(), result.y());
                     }) |
                     ranges::to<std::vector>;
  return update(base_pose_in_odom, std::move(measurement));
}

// Overloaded update method for SparsePointCloud3.
auto Amcl::update(Sophus::SE2d base_pose_in_odom, beluga_ros::SparsePointCloud3f point_cloud)
    -> std::optional<std::pair<Sophus::SE2d, Sophus::Matrix3d>> {
  std::vector<std::pair<double, double>> measurement;  // NOTE: Should be float?
  measurement.reserve(point_cloud.size());

  // Transform points from the sensor frame to the base frame (and project to Z=0).
  const auto project_to_base_xy_plane = [&point_cloud](const auto& p) {
    const auto result = point_cloud.origin() * p.template cast<double>();
    return std::pair{result.x(), result.y()};
  };
  measurement = point_cloud.points() | ranges::views::transform(project_to_base_xy_plane) | ranges::to<std::vector>();

  return update(base_pose_in_odom, std::move(measurement));
}

// Overloaded update method for vector of double pairs.
auto Amcl::update(
    Sophus::SE2d base_pose_in_odom,
    std::vector<std::pair<double, double>>&& measurement)  // NOTE: Should be float?
    -> std::optional<std::pair<Sophus::SE2d, Sophus::Matrix3d>> {
  if (particles_.empty()) {
    return std::nullopt;
  }

  if (!update_policy_(base_pose_in_odom) && !force_update_) {
    return std::nullopt;
  }

  std::visit(
      [&, this](auto& policy, auto& motion_model, auto& sensor_model) {
        particles_ |=
            beluga::actions::propagate(policy, motion_model(control_action_window_ << base_pose_in_odom)) |  //
            beluga::actions::reweight(policy, sensor_model(std::move(measurement))) |                        //
            beluga::actions::normalize(policy);
      },
      execution_policy_, motion_model_, sensor_model_);

  const double random_state_probability = random_probability_estimator_(particles_);

  if (resample_policy_(particles_)) {
    auto random_state = ranges::compose(beluga::make_from_state<particle_type>, std::ref(map_distribution_));

    if (random_state_probability > 0.0) {
      random_probability_estimator_.reset();
    }

    particles_ |= beluga::views::sample |
                  beluga::views::random_intersperse(std::move(random_state), random_state_probability) |
                  beluga::views::take_while_kld(
                      spatial_hasher_,        //
                      params_.min_particles,  //
                      params_.max_particles,  //
                      params_.kld_epsilon,    //
                      params_.kld_z) |
                  beluga::actions::assign;
  }

  force_update_ = false;
  return beluga::cluster_based_estimate(beluga::views::states(particles_), beluga::views::weights(particles_));
}

}  // namespace beluga_ros
