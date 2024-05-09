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

#ifndef BELUGA_ROS_AMCL_HPP
#define BELUGA_ROS_AMCL_HPP

#include <optional>
#include <utility>
#include <variant>

#include <beluga/beluga.hpp>
#include <beluga_ros/laser_scan.hpp>
#include <beluga_ros/occupancy_grid.hpp>

#include <range/v3/view/take_exactly.hpp>

/**
 * \file
 * \brief Generic two-dimensional implementation of the Adaptive Monte Carlo Localization (AMCL) algorithm in 2D.
 */

namespace beluga_ros {

/// Struct containing parameters for the Adaptive Monte Carlo Localization (AMCL) implementation.
struct AmclParams {
  /// \brief Translational movement required from last resample for resampling to happen again.
  double update_min_d = 0.25;

  /// \brief Rotational movement required from last resample for resampling to happen again.
  double update_min_a = 0.2;

  /// \brief Number of filter updates required before resampling.
  std::size_t resample_interval = 1UL;

  /// \brief Whether to enable selective resampling \cite grisetti2007selectiveresampling
  /// to help avoid loss of diversity in the particle population. The resampling
  /// step will only happen if the effective number of particles
  /// (\f$N_{eff} = 1/ {\sum w_i^2}\f$) is lower than half the current number of
  /// particles, where \f$w_i\f$ refers to the normalized weight of each particle.
  bool selective_resampling = false;

  /// \brief Minimum allowed number of particles.
  std::size_t min_particles = 500UL;

  /// \brief Maximum allowed number of particles.
  std::size_t max_particles = 2000UL;

  /// \brief Exponential decay rate for the slow average weight filter, used in deciding when to
  /// recover from a bad approximation by adding random poses \cite thrun2005probabilistic .
  double alpha_slow = 0.001;

  /// \brief Exponential decay rate for the fast average weight filter, used in deciding when to
  /// recover from a bad approximation by adding random poses \cite thrun2005probabilistic .
  double alpha_fast = 0.1;

  /// \brief Maximum particle filter population error between the true distribution and the
  /// estimated distribution. It is used in KLD resampling \cite fox2001adaptivekldsampling
  /// to limit the allowed number of particles to the minimum necessary.
  double kld_epsilon = 0.05;

  /// \brief Upper standard normal quantile for \f$P\f$, where \f$P\f$ is the probability that the error in
  /// the estimated distribution will be less than `kld_epsilon` in KLD resampling \cite fox2001adaptivekldsampling .
  double kld_z = 3.0;

  /// \brief Spatial resolution along the x-axis to create buckets for KLD resampling.
  double spatial_resolution_x = 0.5;

  /// \brief Spatial resolution along the y-axis to create buckets for KLD resampling.
  double spatial_resolution_y = 0.5;

  /// \brief Spatial resolution around the z-axis to create buckets for KLD resampling.
  double spatial_resolution_theta = 10 * Sophus::Constants<double>::pi() / 180;
};

/// Implementation of the 2D Adaptive Monte Carlo Localization (AMCL) algorithm.
/// Generic two-dimensional implementation of the Adaptive Monte Carlo Localization (AMCL) algorithm in 2D.
class Amcl {
 public:
  /// Weighted SE(2) state particle type.
  using particle_type = std::tuple<Sophus::SE2d, beluga::Weight>;

  /// Motion model variant type for runtime selection support.
  using motion_model_variant = std::variant<
      beluga::DifferentialDriveModel,     //
      beluga::OmnidirectionalDriveModel,  //
      beluga::StationaryModel>;

  /// Sensor model variant type for runtime selection support.
  using sensor_model_variant = std::variant<
      beluga::LikelihoodFieldModel<beluga_ros::OccupancyGrid>,  //
      beluga::BeamSensorModel<beluga_ros::OccupancyGrid>>;

  /// Execution policy variant type for runtime selection support.
  using execution_policy_variant = std::variant<std::execution::sequenced_policy, std::execution::parallel_policy>;

  /// Constructor.
  /**
   * \param map Occupancy grid map.
   * \param motion_model Variant of motion model.
   * \param sensor_model Variant of sensor model.
   * \param params Parameters for AMCL implementation.
   * \param execution_policy Variant of execution policy.
   */
  Amcl(
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
        update_policy_{beluga::policies::on_motion(params_.update_min_d, params_.update_min_a)},
        resample_policy_{beluga::policies::every_n(params_.resample_interval)} {
    if (params_.selective_resampling) {
      resample_policy_ = resample_policy_ && beluga::policies::on_effective_size_drop;
    }
  }

  /// Returns a reference to the current set of particles.
  [[nodiscard]] const auto& particles() const { return particles_; }

  /// Initialize particles using a custom distribution.
  template <class Distribution>
  void initialize(Distribution distribution) {
    particles_ = beluga::views::sample(std::move(distribution)) |                    //
                 ranges::views::transform(beluga::make_from_state<particle_type>) |  //
                 ranges::views::take_exactly(params_.max_particles) |                //
                 ranges::to<beluga::TupleVector>;
    force_update_ = true;
  }

  /// Initialize particles with a given pose and covariance.
  /**
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  void initialize(Sophus::SE2d pose, Sophus::Matrix3d covariance) {
    initialize(beluga::MultivariateNormalDistribution{pose, covariance});
  }

  /// Initialize particles using the default map distribution.
  void initialize_from_map() { initialize(std::ref(map_distribution_)); }

  /// Update the map used for localization.
  void update_map(beluga_ros::OccupancyGrid map) {
    map_distribution_ = beluga::MultivariateUniformDistribution{map};
    std::visit([&](auto& sensor_model) { sensor_model.update_map(std::move(map)); }, sensor_model_);
  }

  /// Update particles based on motion and sensor information.
  /**
   * This method performs a particle filter update step using motion and sensor data. It evaluates whether
   * an update is necessary based on the configured update policy and the force_update flag. If an update
   * is required, the motion model and sensor model updates are applied to the particles, and the particle
   * weights are adjusted accordingly. Also, according to the configured resampling policy, the particles
   * are resampled to maintain diversity and prevent degeneracy.
   *
   * \param base_pose_in_odom Base pose in the odometry frame.
   * \param laser_scan Laser scan data.
   * \return An optional pair containing the estimated pose and covariance after the update,
   *         or std::nullopt if no update was performed.
   */
  auto update(Sophus::SE2d base_pose_in_odom, beluga_ros::LaserScan laser_scan)
      -> std::optional<std::pair<Sophus::SE2d, Sophus::Matrix3d>> {
    if (particles_.empty()) {
      return std::nullopt;
    }

    if (!update_policy_(base_pose_in_odom) && !force_update_) {
      return std::nullopt;
    }

    // TODO(nahuel): Remove this once we update the measurement type.
    auto measurement = laser_scan.points_in_cartesian_coordinates() |  //
                       ranges::views::transform([&laser_scan](const auto& p) {
                         const auto result = laser_scan.origin() * Sophus::Vector3d{p.x(), p.y(), 0};
                         return std::make_pair(result.x(), result.y());
                       }) |
                       ranges::to<std::vector>;

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
    return beluga::estimate(beluga::views::states(particles_), beluga::views::weights(particles_));
  }

  /// Force a manual update of the particles on the next iteration of the filter.
  void force_update() { force_update_ = true; }

 private:
  beluga::TupleVector<particle_type> particles_;

  AmclParams params_;
  beluga::MultivariateUniformDistribution<Sophus::SE2d, beluga_ros::OccupancyGrid> map_distribution_;
  motion_model_variant motion_model_;
  sensor_model_variant sensor_model_;
  execution_policy_variant execution_policy_;

  beluga::spatial_hash<Sophus::SE2d> spatial_hasher_;
  beluga::ThrunRecoveryProbabilityEstimator random_probability_estimator_;
  beluga::any_policy<Sophus::SE2d> update_policy_;
  beluga::any_policy<decltype(particles_)> resample_policy_;

  beluga::RollingWindow<Sophus::SE2d, 2> control_action_window_;

  bool force_update_{true};
};

}  // namespace beluga_ros

#endif  // BELUGA_ROS_AMCL_HPP
