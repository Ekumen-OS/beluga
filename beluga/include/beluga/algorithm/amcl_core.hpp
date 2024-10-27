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

#ifndef BELUGA_ALGORITHM_AMCL_CORE_HPP
#define BELUGA_ALGORITHM_AMCL_CORE_HPP

#include <execution>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <beluga/actions/assign.hpp>
#include <beluga/actions/normalize.hpp>
#include <beluga/actions/propagate.hpp>
#include <beluga/actions/reweight.hpp>
#include <beluga/algorithm/estimation.hpp>
#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/containers/circular_array.hpp>
#include <beluga/containers/tuple_vector.hpp>
#include <beluga/policies/every_n.hpp>
#include <beluga/policies/on_effective_size_drop.hpp>
#include <beluga/policies/on_motion.hpp>
#include <beluga/primitives.hpp>
#include <beluga/random/multivariate_normal_distribution.hpp>
#include <beluga/views/random_intersperse.hpp>
#include <beluga/views/sample.hpp>
#include <beluga/views/take_while_kld.hpp>

namespace beluga {

/// Struct containing parameters for the Adaptive Monte Carlo Localization (AMCL) implementation.
struct AmclParams {
  /// Min distance in meters between updates.
  double update_min_d = 0.25;
  /// Min angular distance in radians between updates.
  double update_min_a = 0.2;
  /// Filter iterations interval at which a resampling will happen, unitless.
  std::size_t resample_interval = 1UL;
  /// Whether to use selective resampling or not.
  bool selective_resampling = false;
  /// Minimum number of particles in the filter at any point in time.
  std::size_t min_particles = 500UL;
  /// Maximum number of particles in the filter at any point in time.
  std::size_t max_particles = 2000UL;
  /// Used as part of the kld sampling mechanism.
  double kld_epsilon = 0.05;
  /// Used as part of the kld sampling mechanism.
  double kld_z = 3.0;
};

/// Implementation of the Adaptive Monte Carlo Localization (AMCL) algorithm.
/**
 * \tparam MotionModel Class representing a motion model. Must satisfy \ref MotionModelPage.
 * \tparam SensorModel Class representing a sensor model. Must satisfy \ref SensorModelPage.
 * \tparam ParticleType Full particle type, containing state, weight and possibly
 * other information .
 */
template <
    class MotionModel,
    class SensorModel,
    class Particle = std::tuple<typename SensorModel::state_type, beluga::Weight>>
class Amcl {
  using particle_type = Particle;
  using measurement_type = typename SensorModel::measurement_type;
  using state_type = typename SensorModel::state_type;
  using map_type = typename SensorModel::map_type;
  using spatial_hasher_type = spatial_hash<state_type>;
  using estimation_type = std::invoke_result_t<beluga::detail::estimate_fn, std::vector<state_type>>;

 public:
  /// Construct a AMCL instance.
  /**
   * \param motion_model Motion model instance.
   * \param sensor_model Sensor model Instance.
   * \param spatial_hasher A spatial hasher instance capable of computing a hash out of a particle state.
   * \param params Parameters for AMCL implementation.
   */
  Amcl(
      MotionModel motion_model,
      SensorModel sensor_model,
      spatial_hasher_type spatial_hasher,
      const AmclParams& params = AmclParams{})
      : params_{params},
        motion_model_{std::move(motion_model)},
        sensor_model_{std::move(sensor_model)},
        spatial_hasher_{std::move(spatial_hasher)},
        update_policy_{beluga::policies::on_motion<state_type>(params_.update_min_d, params_.update_min_a)},
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
   * \tparam CovarianceT type representing a covariance, compliant with state_type.
   * \throw std::runtime_error If the provided covariance is invalid.
   */
  template <class Covariance>
  void initialize(state_type pose, Covariance covariance) {
    initialize(beluga::MultivariateNormalDistribution{std::move(pose), std::move(covariance)});
  }

  /// Update the map used for localization.
  void update_map(map_type map) { sensor_model_.update_map(std::move(map)); }

  /// Update particles based on motion and sensor information.
  /**
   * This method performs a particle filter update step using motion and sensor data. It evaluates whether
   * an update is necessary based on the configured update policy and the force_update flag. If an update
   * is required, or if it was forced  via `force_update()`, the motion model and sensor model updates are applied to
   * the particles, and the particle weights are adjusted accordingly. Also, according to the configured resampling
   * policy, the particles are resampled to maintain diversity and prevent degeneracy.
   *
   * \param control_action Control action.
   * \param measurement Measurement data.
   * \return An optional pair containing the estimated pose and covariance after the update,
   *         or std::nullopt if no update was performed.
   */
  auto update(state_type control_action, measurement_type measurement) -> std::optional<estimation_type> {
    if (particles_.empty()) {
      return std::nullopt;
    }

    if (!update_policy_(control_action) && !force_update_) {
      return std::nullopt;
    }

    particles_ |= beluga::actions::propagate(motion_model_(control_action_window_ << std::move(control_action))) |  //
                  beluga::actions::reweight(sensor_model_(std::move(measurement))) |                                //
                  beluga::actions::normalize;

    if (resample_policy_(particles_)) {
      particles_ |= beluga::views::sample |
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

  MotionModel motion_model_;
  SensorModel sensor_model_;

  spatial_hasher_type spatial_hasher_;
  beluga::any_policy<state_type> update_policy_;
  beluga::any_policy<decltype(particles_)> resample_policy_;

  beluga::RollingWindow<state_type, 2> control_action_window_;

  bool force_update_{true};
};

}  // namespace beluga

#endif  // BELUGA_ALGORITHM_AMCL_CORE_HPP
