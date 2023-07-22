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

#ifndef BELUGA_AMCL_FILTER_UPDATE_CONTROL_FILTER_UPDATE_CONTROL_MIXIN_HPP
#define BELUGA_AMCL_FILTER_UPDATE_CONTROL_FILTER_UPDATE_CONTROL_MIXIN_HPP

#include <execution>
#include <utility>
#include <vector>

#include <beluga_amcl/tf2_sophus.hpp>

#include <ciabatta/ciabatta.hpp>

/**
 * \file
 * \brief Implementation of a filter update control algorithm based on Nav2's.
 */

namespace beluga_amcl {

/// \brief Public interface of the FilterUpdateControl filter customization mixin
class FilterUpdateControlInterface {
 public:
  using motion_update_type = Sophus::SE2d;
  using sensor_update_type = std::vector<std::pair<double, double>>;

  /// \brief Default destructor
  virtual ~FilterUpdateControlInterface() = default;

  /// @brief Execute a filter update step, conditioned by the installed resampling policies.
  /**
   * @param odom_to_base_transform Odom-to-base transform, for the motion update.
   * @param laser_scan Laser-scan information, for the sensor update.
   * @return true if the pose estimate can be expected to be different after this update.
   */
  virtual bool update_filter(
      const motion_update_type& odom_to_base_transform,
      sensor_update_type&& laser_scan,
      bool force_update) = 0;

  /** \overload
   * It allows specifying a sequenced execution policy.
   */
  virtual bool update_filter(
      std::execution::sequenced_policy exec_policy,
      const motion_update_type& odom_to_base_transform,
      sensor_update_type&& laser_scan,
      bool force_update) = 0;

  /** \overload
   * It allows specifying a parallel execution policy.
   */
  virtual bool update_filter(
      std::execution::parallel_policy exec_policy,
      const motion_update_type& odom_to_base_transform,
      sensor_update_type&& laser_scan,
      bool force_update) = 0;
};

/// Resampling policy poller.
/**
 * \tparam Mixin The mixed-in type.
 * \tparam UpdateFilterWhenMovingPolicy The resample on motion policy type.
 * \tparam ResampleIntervalPolicy The resample interval policy type.
 * \tparam SelectiveResamplingPolicy The selective resampling policy type.
 */
template <
    typename Mixin,
    typename UpdateFilterWhenMovingPolicy,
    typename ResampleIntervalPolicy,
    typename SelectiveResamplingPolicy>
class FilterUpdateControlMixin : public Mixin {
 public:
  using motion_update_type = Sophus::SE2d;
  using sensor_update_type = std::vector<std::pair<double, double>>;

  /// Constructs a FilterUpdateControlMixin instance.
  /**
   * \tparam ...Rest Arguments types for the remaining mixin constructors.
   * \param policy_configs Configuration parameters for each of the installed resampling policies.
   * \param ...rest Arguments that are not used by this part of the mixin, but by others.
   */
  template <typename... Rest>
  explicit FilterUpdateControlMixin(
      const typename UpdateFilterWhenMovingPolicy::param_type& resample_on_motion_config,
      const typename ResampleIntervalPolicy::param_type& resample_interval_config,
      const typename SelectiveResamplingPolicy::param_type& selective_resampling_config,
      Rest&&... rest)
      : Mixin(std::forward<Rest>(rest)...),
        update_when_moving_policy_{resample_on_motion_config},
        resample_interval_policy_{resample_interval_config},
        selective_resampling_policy_{selective_resampling_config} {}

  /// \brief Execute a filter update step, conditioned by the installed resampling policies.
  /**
   * @param odom_to_base_transform Odom-to-base transform, for the motion update.
   * @param laser_scan Laser-scan information, for the sensor update.
   * @param force_update If true, the filter will be updated regardless of the resampling policies.
   * @return true if the pose estimate can be expected to be different after this update.
   */
  [[nodiscard]] bool update_filter(
      const motion_update_type& odom_to_base_transform,
      sensor_update_type&& laser_scan,
      bool force_update) final {
    return this->update_filter_impl(std::execution::seq, odom_to_base_transform, std::move(laser_scan), force_update);
  }

  /**
   * \overload
   * Overload for performing a full filter update using the sequential execution policy.
   */
  [[nodiscard]] bool update_filter(
      std::execution::sequenced_policy exec_policy,
      const motion_update_type& odom_to_base_transform,
      sensor_update_type&& laser_scan,
      bool force_update) final {
    return this->update_filter_impl(exec_policy, odom_to_base_transform, std::move(laser_scan), force_update);
  }

  /**
   * \overload
   * Overload for performing a full filter update using the parallel execution policy.
   */
  [[nodiscard]] bool update_filter(
      std::execution::parallel_policy exec_policy,
      const motion_update_type& odom_to_base_transform,
      sensor_update_type&& laser_scan,
      bool force_update) final {
    return this->update_filter_impl(exec_policy, odom_to_base_transform, std::move(laser_scan), force_update);
  }

 private:
  UpdateFilterWhenMovingPolicy update_when_moving_policy_;
  ResampleIntervalPolicy resample_interval_policy_;
  SelectiveResamplingPolicy selective_resampling_policy_;

  template <typename ExecutionPolicy>
  [[nodiscard]] bool update_filter_impl(
      ExecutionPolicy exec_policy,
      const motion_update_type& odom_to_base_transform,
      sensor_update_type&& laser_scan,
      bool force_update) {
    bool update_filter_estimate = false;

    // based on NAV2 AMCL's decision tree to determine when to update, resample and
    // update the filter estimates.

    const auto far_enough_to_update = update_when_moving_policy_.do_filter_update(odom_to_base_transform);

    if (force_update || far_enough_to_update) {
      this->self().update_motion(odom_to_base_transform);
      this->self().sample(exec_policy);

      this->self().update_sensor(std::move(laser_scan));
      this->self().reweight(exec_policy);

      const auto resample_rate_divider_agrees = resample_interval_policy_.do_resampling();

      if (resample_rate_divider_agrees) {
        // Nav2 updates the filter estimates regardless of whether selective resampling
        // actually resamples or not
        update_filter_estimate = true;

        const auto selective_resampling_policy_agrees = selective_resampling_policy_.do_resampling(this->self());

        if (selective_resampling_policy_agrees) {
          this->self().resample();
        }
      }
    }

    return update_filter_estimate || force_update;
  }
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL_FILTER_UPDATE_CONTROL_FILTER_UPDATE_CONTROL_MIXIN_HPP
