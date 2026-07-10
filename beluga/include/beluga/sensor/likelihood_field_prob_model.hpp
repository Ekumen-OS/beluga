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

#ifndef BELUGA_SENSOR_LIKELIHOOD_FIELD_PROB_MODEL_HPP
#define BELUGA_SENSOR_LIKELIHOOD_FIELD_PROB_MODEL_HPP

#include <algorithm>
#include <beluga/sensor/likelihood_field_model_base.hpp>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include <sophus/common.hpp>

/**
 * \file
 * \brief Implementation of a likelihood field prob sensor model for range finders.
 */

namespace beluga {

/// Parameters used to construct a LikelihoodFieldProbModel instance.
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.4, particularly Table 6.3.
 *
 * The first five fields mirror beluga::LikelihoodFieldModelBaseParam (preserving its
 * field order for aggregate initialization). The remaining fields configure the optional
 * beam skipping heuristic, which detects beams that disagree with the map across a large
 * fraction of the particle set (e.g. caused by unmapped/dynamic obstacles) and excludes
 * them from the weight computation. Beam skipping is disabled by default; see
 * https://github.com/Ekumen-OS/beluga/issues/187 for context.
 */
struct LikelihoodFieldProbModelParam {
  /// Maximum distance to obstacle.
  double max_obstacle_distance = 100.0;
  /// Maximum range of a laser ray.
  double max_laser_distance = 2.0;
  /// Weight used to combine the probability of hitting an obstacle.
  double z_hit = 0.5;
  /// Weight used to combine the probability of random noise in perception.
  double z_random = 0.5;
  /// Standard deviation of a gaussian centered arounds obstacles.
  double sigma_hit = 0.2;
  /// Whether to enable the beam skipping heuristic.
  bool do_beamskip = false;
  /// Distance to the nearest obstacle below which a beam is considered to agree with the map.
  double beam_skip_distance = 0.5;
  /// Fraction of particles that must agree on a beam for it to be kept.
  double beam_skip_threshold = 0.3;
  /// If the fraction of skipped beams exceeds this value, skipping is disabled for the update.
  double beam_skip_error_threshold = 0.9;
};

/// Likelihood field prob sensor model for range finders.
/**
 * @copydoc LikelihoodFieldModelBase
 */
template <class OccupancyGrid>
class LikelihoodFieldProbModel : public LikelihoodFieldModelBase<OccupancyGrid> {
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
  using param_type = LikelihoodFieldProbModelParam;

  /// Constructs a LikelihoodFieldProbModel instance.
  /**
   * \param params Parameters to configure this instance.
   *  See beluga::LikelihoodFieldProbModelParam for details.
   * \param grid Occupancy grid representing the static map that the sensor model
   *  uses to compute a likelihood field for lidar hits and compute importance weights
   *  for particle states.
   */
  explicit LikelihoodFieldProbModel(const param_type& params, const map_type& grid)
      : LikelihoodFieldModelBase<OccupancyGrid>(to_base_param(params), grid),
        do_beamskip_{params.do_beamskip},
        beam_skip_threshold_{params.beam_skip_threshold},
        beam_skip_error_threshold_{params.beam_skip_error_threshold},
        likelihood_threshold_{compute_likelihood_threshold(params)} {}

  /// Precomputes the beam skipping mask from the current particle states.
  /**
   * Runs the first pass of the beam skipping heuristic: for every beam, it counts the
   * fraction of particle states for which the beam endpoint lands close enough to a mapped
   * obstacle (within `beam_skip_distance`, expressed here as a likelihood threshold). Beams
   * for which that fraction falls below `beam_skip_threshold` are masked out and ignored by
   * the subsequent weight computation in `operator()`. If skipping would discard more than
   * `beam_skip_error_threshold` of the beams, the mask is reset so that all beams are used,
   * preventing filter divergence. This is a no-op when beam skipping is disabled.
   *
   * Must be called once per update, after motion propagation and before reweighting.
   *
   * \tparam StateRange A range of particle states (Sophus::SE2d).
   * \param points 2D lidar hit points in the reference frame of particle states.
   * \param states Range with the (propagated) particle states for the current update.
   */
  template <class StateRange>
  void prepare(const measurement_type& points, StateRange&& states) {
    if (!do_beamskip_) {
      return;
    }

    const std::size_t num_beams = points.size();
    beam_mask_.assign(num_beams, true);
    if (num_beams == 0) {
      return;
    }

    const auto unknown_space_occupancy_prob = static_cast<float>(1. / this->params_.max_laser_distance);
    std::vector<std::size_t> obs_count(num_beams, 0);
    std::size_t num_states = 0;

    for (const auto& state : states) {
      ++num_states;
      const auto transform = this->world_to_likelihood_field_transform_ * state;
      const auto x_offset = transform.translation().x();
      const auto y_offset = transform.translation().y();
      const auto cos_theta = transform.so2().unit_complex().x();
      const auto sin_theta = transform.so2().unit_complex().y();
      for (std::size_t i = 0; i < num_beams; ++i) {
        const auto& point = points[i];
        const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
        const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
        const auto pz = this->likelihood_field_.data_near(x, y).value_or(unknown_space_occupancy_prob);
        // The likelihood field is monotonically decreasing in the distance to the nearest
        // obstacle, so "distance < beam_skip_distance" is equivalent to "pz > threshold".
        if (pz > likelihood_threshold_) {
          ++obs_count[i];
        }
      }
    }

    if (num_states == 0) {
      return;
    }

    std::size_t skipped = 0;
    for (std::size_t i = 0; i < num_beams; ++i) {
      const double ratio = static_cast<double>(obs_count[i]) / static_cast<double>(num_states);
      beam_mask_[i] = ratio > beam_skip_threshold_;
      if (!beam_mask_[i]) {
        ++skipped;
      }
    }

    // Safety fallback: if too many beams would be skipped, integrate all of them instead.
    const double skipped_ratio = static_cast<double>(skipped) / static_cast<double>(num_beams);
    if (skipped_ratio > beam_skip_error_threshold_) {
      std::fill(beam_mask_.begin(), beam_mask_.end(), true);
    }
  }

  /// Returns the current beam skipping mask (one boolean per beam, true means the beam is used).
  /**
   * The mask is populated by `prepare()`. It is empty until the first call, which is
   * equivalent to using every beam. Mainly useful for introspection and testing.
   */
  [[nodiscard]] const std::vector<bool>& beam_mask() const { return beam_mask_; }

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

      double log_weight = 0.0;
      for (std::size_t i = 0; i < points.size(); ++i) {
        // Skip beams that were masked out by prepare(). When beam skipping is disabled (or
        // prepare() was never called) the mask is not consulted and every beam contributes,
        // reproducing the plain likelihood field prob behavior.
        if (do_beamskip_ && i < beam_mask_.size() && !beam_mask_[i]) {
          continue;
        }
        // Transform the end point of the laser to the grid local coordinate system.
        // Not using Eigen/Sophus because they make the routine x10 slower.
        // See `benchmark_likelihood_field_model.cpp` for reference.
        const auto& point = points[i];
        const auto x = point.first * cos_theta - point.second * sin_theta + x_offset;
        const auto y = point.first * sin_theta + point.second * cos_theta + y_offset;
        const auto pz =
            static_cast<double>(this->likelihood_field_.data_near(x, y).value_or(unknown_space_occupancy_prob));
        log_weight += std::log(pz);
      }
      return std::exp(log_weight);
    };
  }

 private:
  bool do_beamskip_;                  ///< Whether the beam skipping heuristic is enabled.
  double beam_skip_threshold_;        ///< Fraction of particles that must agree for a beam to be kept.
  double beam_skip_error_threshold_;  ///< Skipped-beam fraction above which skipping is disabled.
  float likelihood_threshold_;        ///< Likelihood equivalent of `beam_skip_distance`.
  std::vector<bool> beam_mask_;       ///< Per-beam mask computed by `prepare()` (true means used).

  /// Builds the base model parameters from the full prob model parameters.
  static LikelihoodFieldModelBaseParam to_base_param(const param_type& params) {
    LikelihoodFieldModelBaseParam base;
    base.max_obstacle_distance = params.max_obstacle_distance;
    base.max_laser_distance = params.max_laser_distance;
    base.z_hit = params.z_hit;
    base.z_random = params.z_random;
    base.sigma_hit = params.sigma_hit;
    return base;
  }

  /// Converts `beam_skip_distance` into the equivalent likelihood field value.
  /**
   * Uses the same gaussian the base class uses to build the likelihood field, so that the
   * "distance to obstacle < beam_skip_distance" agreement test can be evaluated directly on
   * the precomputed likelihood field without keeping the distance map around.
   */
  static float compute_likelihood_threshold(const param_type& params) {
    const double two_squared_sigma = 2.0 * params.sigma_hit * params.sigma_hit;
    const double amplitude = params.z_hit / (params.sigma_hit * std::sqrt(2.0 * Sophus::Constants<double>::pi()));
    const double offset = params.z_random / params.max_laser_distance;
    const double squared_distance = params.beam_skip_distance * params.beam_skip_distance;
    return static_cast<float>(amplitude * std::exp(-squared_distance / two_squared_sigma) + offset);
  }
};

}  // namespace beluga

#endif
