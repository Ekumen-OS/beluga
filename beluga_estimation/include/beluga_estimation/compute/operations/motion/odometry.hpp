// Copyright 2026 Ekumen, Inc.
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

#pragma once

/**
 * @file
 * @brief Defines the SE2OdometryConnector operation for SE(2) pose graph constraints.
 */

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>

#include <Eigen/Core>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/algebra/interpolation.hpp>
#include <beluga_estimation/common/probability.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/constraints/se2_pose_kinematic_constraint.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/middleware/subscriber.hpp>
#include <beluga_estimation/msg/odometry.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace beluga_estimation {

/**
 * @brief Bayesian operation that adds SE(2) kinematic constraints between pose variables using odometry.
 *
 * Subscribes to odometry messages and maintains a time-ordered buffer of poses.
 * On each execution, queries the DBN for consecutive `SE2Variable_<T>` nodes
 * and connects each pair with a `SE2PoseKinematicConstraint_` encoding the relative
 * transform observed between their timestamps.
 *
 * Constraints are named `<variable_name>/odometry` so that several connectors
 * can coexist in one DBN (e.g. for multi-agent estimation). The configured device
 * name qualifies log messages only; it does not participate in constraint identity.
 *
 * @tparam T Scalar type (float or double).
 */
template <typename T>
class SE2OdometryConnector_ : public BayesianOperation_<T> {
 public:
  /// @brief Time-stamped odometry snapshot stored in the buffer.
  struct CachedEntry {
    beluga_estimation::SE2<T> pose;                     ///< SE(2) pose in the odometry frame.
    beluga_estimation::Matrix3<T> velocity_covariance;  ///< SE(2) velocity noise covariance for [vx, vy, omega_z].
  };

  /// @brief Configuration for the odometry connector.
  struct Configuration {
    static constexpr double kDefaultMaxBufferAgeSeconds = 5.0;

    /**
     * @brief Loads configuration from middleware parameters.
     * @param middleware The middleware interface for parameter access.
     * @param prefix The parameter namespace prefix.
     * @return A fully populated Configuration instance.
     */
    static Configuration LoadFromParams(
        std::shared_ptr<MiddlewareInterface> middleware,
        const std::string& prefix = "") {
      Configuration config;
      config.device_name = middleware->DeclareParameter<std::string>(prefix + "device_name", "odom");
      config.topic_name = middleware->DeclareParameter<std::string>(prefix + "topic_name", "odom");
      config.variable_name = middleware->DeclareParameter<std::string>(prefix + "variable_name", "pose");
      config.max_buffer_age = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
          middleware->DeclareParameter<double>(prefix + "max_buffer_age", kDefaultMaxBufferAgeSeconds)));
      return config;
    }

    std::string device_name;                  ///< Unique identifier for this odometry source.
    std::string topic_name;                   ///< Topic to subscribe to.
    std::string variable_name;                ///< Name of the SE2Variables to connect in the DBN.
    std::chrono::nanoseconds max_buffer_age;  ///< Maximum age of retained buffer entries.
  };

  /**
   * @brief Creates the connector from middleware parameters.
   * @param middleware The middleware interface.
   * @param prefix The parameter namespace prefix.
   * @return A result containing the connector instance.
   */
  static Result_<std::unique_ptr<SE2OdometryConnector_<T>>> Make(
      std::shared_ptr<MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<SE2OdometryConnector_<T>>(std::move(middleware), prefix);
  }

  /**
   * @brief Constructs the connector from middleware parameters.
   * @param middleware The middleware interface.
   * @param prefix The parameter namespace prefix.
   */
  explicit SE2OdometryConnector_(std::shared_ptr<MiddlewareInterface> middleware, std::string_view prefix = "")
      : SE2OdometryConnector_(middleware, Configuration::LoadFromParams(middleware, std::string(prefix))) {}

  /**
   * @brief Constructs the connector from an explicit configuration.
   * @param middleware The middleware interface.
   * @param config The connector configuration.
   */
  SE2OdometryConnector_(std::shared_ptr<MiddlewareInterface> middleware, Configuration config)
      : middleware_(std::move(middleware)), config_(std::move(config)) {
    // Derive the constraint name from the configured variable name so that several
    // connectors can coexist in one DBN without colliding on (name, timestamp).
    // The device name remains a logging qualifier only.
    constraint_name_ = config_.variable_name + "/odometry";
  }

  /**
   * @brief Subscribes to the odometry topic.
   * @param budget Unused.
   * @return Success, or an error if the subscription could not be created.
   */
  Result Initialize(const ComputeBudget& /*budget*/) override {
    auto subscription = middleware_->CreateSubscriber<beluga_estimation::msg::Odometry>(
        config_.topic_name, [this](std::shared_ptr<const beluga_estimation::msg::Odometry> msg) {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          incoming_queue_.push(std::move(msg));
        });
    if (!subscription) {
      return Err(CollectError(subscription));
    }
    subscription_ = std::move(*subscription);
    return Ok();
  }

  /**
   * @brief Unsubscribes and resets all state.
   * @param budget Unused.
   * @return Success.
   */
  Result Finalize(const ComputeBudget& /*budget*/) override {
    subscription_.reset();
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      incoming_queue_ = {};
    }
    buffer_.clear();
    last_connected_timestamp_.reset();
    warned_anchor_age_ = false;
    return Ok();
  }

  /**
   * @brief Connects consecutive SE2Variables in the DBN with odometry constraints.
   * @param context Execution context with DBN and estimate store.
   * @param budget Resource limits for this execution.
   * @return Success if at least one pair was connected, an error with `Error::kBusy` otherwise.
   */
  Result Execute(Context_<T>& context, const ComputeBudget& budget) override {
    DrainQueueToBuffer();
    EvictBuffer(middleware_->GetClockInterface()->Now());

    if (buffer_.empty()) {
      return Err(Error::kBusy);
    }

    const auto interval =
        last_connected_timestamp_.has_value() ? TimeInterval::From(*last_connected_timestamp_) : TimeInterval{};

    bool processed = false;
    std::size_t num_connected_pairs = 0;
    std::shared_ptr<const SE2Variable_<T>> prev_var;

    for (auto&& var_result : context.dbn.template GetVariables<SE2Variable_<T>>(config_.variable_name, interval)) {
      if (!var_result) {
        return Err(CollectError(var_result));
      }
      auto var = Unwrap(var_result);
      if (!prev_var) {
        prev_var = var;
        continue;
      }

      const Result status = ConnectPair(context, *prev_var, *var);
      if (!status && status.error() == Error::kBusy) {
        break;
      }

      // Advance the anchor unconditionally so a bad pair (kFailure) does not stall
      // all subsequent pairs on every future Execute() call.
      processed = true;
      last_connected_timestamp_ = var->Id().timestamp;
      prev_var = var;

      if (!status) {
        continue;
      }

      ++num_connected_pairs;

      Result budget_status = budget.Check(middleware_->GetClockInterface()->Now());
      if (!budget_status) {
        middleware_->Debug(
            "SE2OdometryConnector: connected {} pose pair(s) from device '{}' before the budget expired",
            num_connected_pairs, config_.device_name);
        return budget_status;
      }
    }

    if (num_connected_pairs > 0) {
      middleware_->Debug(
          "SE2OdometryConnector: connected {} pose pair(s) from device '{}'", num_connected_pairs, config_.device_name);
    }

    return processed ? Ok() : Err(Error::kBusy);
  }

  /**
   * @brief Returns the buffered odometry entry at timestamp `t`, interpolating if needed.
   *
   * Uses geodesic SE(2) interpolation for the pose and linear interpolation for the
   * velocity covariance when `t` falls between two buffered entries.
   *
   * @param t Target timestamp.
   * @return Interpolated entry, or `std::nullopt` if `t` is outside the buffered range.
   */
  std::optional<CachedEntry> LookupAt(const TimePoint& t) const {
    if (buffer_.empty()) {
      return std::nullopt;
    }

    auto it = buffer_.lower_bound(t);

    if (it == buffer_.end()) {
      return std::nullopt;
    }

    if (it->first == t) {
      return it->second;
    }

    if (it == buffer_.begin()) {
      return std::nullopt;
    }

    const auto next_it = it;
    const auto prev_it = std::prev(it);

    const T total = DurationToScalar(next_it->first - prev_it->first);
    if (total <= T(0)) {
      // Defends against a map-ordering invariant being violated upstream; should be unreachable.
      middleware_->Warn(
          "SE2OdometryConnector: odometry buffer has non-monotonic or duplicate timestamps — skipping interpolation");
      return std::nullopt;
    }

    const T elapsed = DurationToScalar(t - prev_it->first);
    const T alpha = elapsed / total;

    const auto& cov_a = prev_it->second.velocity_covariance;
    const auto& cov_b = next_it->second.velocity_covariance;
    return CachedEntry{
        InterpolateSE2Pose(prev_it->second.pose, next_it->second.pose, alpha),
        InterpolateSPD(cov_a, cov_b, alpha),
    };
  }

 private:
  template <typename Duration>
  static T DurationToScalar(const Duration& d) {
    return std::chrono::duration_cast<std::chrono::duration<T>>(d).count();
  }

  void DrainQueueToBuffer() {
    std::queue<std::shared_ptr<const beluga_estimation::msg::Odometry>> local;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      std::swap(local, incoming_queue_);
    }
    while (!local.empty()) {
      const auto& msg = local.front();
      const auto t = msg->GetTimestamp();
      buffer_.insert_or_assign(
          t, CachedEntry{
                 beluga_estimation::ToSE2(msg->GetPose().template cast<T>()),
                 beluga_estimation::ToSE2Covariance(msg->GetTwistCovariance().template cast<T>()),
             });
      local.pop();
    }
  }

  void EvictBuffer(TimePoint now) {
    const auto max_age_cutoff = now - config_.max_buffer_age;

    if (last_connected_timestamp_.has_value()) {
      // Retain one entry before the anchor so LookupAt can interpolate
      // when the anchor timestamp falls between two buffer entries.
      auto keep_from = buffer_.lower_bound(*last_connected_timestamp_);
      if (keep_from != buffer_.begin())
        --keep_from;
      buffer_.erase(buffer_.begin(), keep_from);

      if (!buffer_.empty() && buffer_.begin()->first < max_age_cutoff) {
        if (!warned_anchor_age_) {
          middleware_->Warn("SE2OdometryConnector: odometry buffer has entries older than max_buffer_age");
          warned_anchor_age_ = true;
        }
      } else {
        warned_anchor_age_ = false;
      }
    } else {
      if (!buffer_.empty() && buffer_.begin()->first < max_age_cutoff) {
        middleware_->Warn("SE2OdometryConnector: discarding odometry buffer entries older than max_buffer_age");
        buffer_.erase(buffer_.begin(), buffer_.lower_bound(max_age_cutoff));
      }
    }
  }

  Result ConnectPair(Context_<T>& context, const SE2Variable_<T>& pose_i_var, const SE2Variable_<T>& pose_j_var) {
    const auto t_i = pose_i_var.Id().timestamp;
    const auto t_j = pose_j_var.Id().timestamp;

    const auto entry_i = LookupAt(t_i);
    const auto entry_j = LookupAt(t_j);

    if (!entry_i || !entry_j) {
      return Err(Error::kBusy);
    }

    const beluga_estimation::SE2<T> delta = entry_i->pose.inverse() * entry_j->pose;

    // Q = avg(Σ_vel_i, Σ_vel_j) * dt. Assumes velocity covariances are continuous-time
    // power spectral densities [units²/s]; approximation if driver reports discrete variances.
    // Ref: SLAM Handbook (Barfoot et al.), Chapter 12, Eq. (12.34).
    const T dt = DurationToScalar(t_j - t_i);
    const beluga_estimation::Matrix3<T> Q =
        (entry_i->velocity_covariance + entry_j->velocity_covariance) * (T(0.5) * dt);

    if (Q.llt().info() != Eigen::Success) {
      middleware_->Warn(
          "SE2OdometryConnector: odometry velocity covariance Q is not positive definite — check driver velocity "
          "covariances");
      return Err(Error::kFailure, "odometry velocity covariance is not positive definite");
    }

    // t_j alone is unique: each variable is only ever the later endpoint of one such pair.
    const auto constraint = context.dbn.template Add<SE2PoseKinematicConstraint_<T>>(
        Identifier{constraint_name_, t_j}, pose_i_var.Id(), pose_j_var.Id(), Q, delta);

    if (!constraint) {
      middleware_->Warn("SE2OdometryConnector: odometry constraint already exists for this pair — skipping");
      return Ok();
    }

    return Ok();
  }

  Configuration config_;
  std::shared_ptr<MiddlewareInterface> middleware_;
  std::shared_ptr<Subscriber<beluga_estimation::msg::Odometry>> subscription_;
  std::mutex queue_mutex_;
  std::queue<std::shared_ptr<const beluga_estimation::msg::Odometry>> incoming_queue_;
  std::map<TimePoint, CachedEntry> buffer_;
  std::optional<TimePoint> last_connected_timestamp_;
  bool warned_anchor_age_ = false;
  std::string constraint_name_;
};

using SE2OdometryConnector = SE2OdometryConnector_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation::SE2OdometryConnector, beluga_estimation::BayesianOperation);
