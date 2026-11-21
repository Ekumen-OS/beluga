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
 * @brief Defines the SE2PoseBroadcaster operation for publishing SE(2) poses.
 */

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <fmt/format.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace beluga_estimation {

/**
 * @brief Bayesian operation that broadcasts SE(2) poses from the network.
 *
 * This operation queries an SE2Variable from the Dynamic Bayesian Network
 * by name and broadcasts/publishes it. The broadcasting mechanism (transform,
 * topic, or both) can be configured via middleware parameters.
 *
 * Expected parameters under the given prefix:
 * - `<prefix>variable_name`: Name of the SE2Variable to broadcast (string)
 * - `<prefix>max_broadcast_rate`: Maximum broadcast rate in Hz (double, default: 10.0)
 * - `<prefix>target_frame`: Target frame for the transform (string)
 * - `<prefix>source_frame`: Source frame for the transform (string)
 * - `<prefix>target_estimate`: Name of the estimate to use (string, default: auto-detect)
 *
 * @tparam T Scalar type used by the filter (e.g., `float` or `double`).
 */
template <typename T>
class SE2PoseBroadcaster_ : public BayesianOperation_<T> {
 public:
  /**
   * @brief Configuration structure for the SE2 pose broadcaster.
   *
   * Specifies which variable to broadcast and the broadcast settings.
   */
  struct Configuration {
    /**
     * @brief Loads broadcaster configuration from middleware parameters.
     *
     * Expects the following parameters under the given prefix:
     * - `<prefix>variable_name`: Name of the SE2Variable to broadcast
     * - `<prefix>max_broadcast_rate`: Maximum broadcast rate in Hz
     * - `<prefix>target_frame`: Target frame for the transform
     * - `<prefix>source_frame`: Source frame for the transform
     * - `<prefix>target_estimate`: Name of the estimate to use
     *
     * @param middleware The middleware interface for parameter access.
     * @param prefix The parameter namespace prefix (e.g., "broadcaster.").
     * @return A fully configured Configuration instance.
     */
    static Configuration LoadFromParams(
        std::shared_ptr<MiddlewareInterface> middleware,
        const std::string& prefix = "") {
      Configuration config;
      config.variable_name = middleware->DeclareParameter<std::string>(prefix + "variable_name");
      // Default max broadcast rate to 0, which means disabled.
      config.max_broadcast_rate = middleware->DeclareParameter<double>(prefix + "max_broadcast_rate", 0.0);
      config.target_frame = middleware->DeclareParameter<std::string>(prefix + "target_frame");
      config.source_frame = middleware->DeclareParameter<std::string>(prefix + "source_frame");
      config.target_estimate = middleware->DeclareParameter<std::string>(prefix + "target_estimate", "");
      return config;
    }

    /// Name of the SE2Variable to query from the DBN.
    std::string variable_name;
    /// Maximum broadcast rate in Hz.
    double max_broadcast_rate;
    /// Target frame for the transform broadcast.
    std::string target_frame;
    /// Source frame for the transform broadcast.
    std::string source_frame;
    /// Optional name of a specific estimate to use. Auto-detect if unset.
    std::string target_estimate;
  };

  /**
   * @brief Creates the pose broadcaster from middleware parameters.
   *
   * @param middleware The middleware interface for parameter access and publishing.
   * @param prefix The parameter namespace prefix for this operation (e.g., "broadcaster.").
   * @return A result containing the broadcaster instance.
   */
  static Result_<std::unique_ptr<SE2PoseBroadcaster_<T>>> Make(
      std::shared_ptr<MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<SE2PoseBroadcaster_<T>>(std::move(middleware), prefix);
  }

  /**
   * @brief Constructs the pose broadcaster from middleware parameters.
   *
   * Loads the configuration using `Configuration::LoadFromParams()` and
   * initializes the broadcaster.
   *
   * @param middleware The middleware interface for parameter access and publishing.
   * @param prefix The parameter namespace prefix for this operation (e.g., "broadcaster.").
   */
  explicit SE2PoseBroadcaster_(std::shared_ptr<MiddlewareInterface> middleware, std::string_view prefix)
      : SE2PoseBroadcaster_(middleware, Configuration::LoadFromParams(middleware, std::string(prefix))) {}

  /**
   * @brief Constructs the pose broadcaster from an explicit configuration.
   *
   * @param middleware The middleware interface for publishing.
   * @param config The broadcaster configuration.
   */
  explicit SE2PoseBroadcaster_(std::shared_ptr<MiddlewareInterface> middleware, Configuration config)
      : middleware_(std::move(middleware)), config_(std::move(config)) {
    // Calculate the minimum time period between broadcasts
    if (config_.max_broadcast_rate > 0.0) {
      broadcast_period_opt_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.max_broadcast_rate));
    }
  }

  /**
   * @brief Allocates resources and performs one-time setup.
   *
   * @param budget Resource limits for initialization.
   * @return Success.
   */
  Result Initialize(const ComputeBudget& budget) override { return Ok(); }

  /**
   * @brief Executes the broadcasting operation.
   *
   * Fetches the latest SE(2) variable from the DBN, resolves an on-manifold estimate
   * (either by @ref Configuration::target_estimate or by automatic variable lookup),
   * extracts the distribution, and broadcasts the pose transform.
   *
   * @param context Execution context with DBN and estimate store.
   * @param budget Resource limits for this execution.
   * @return Success, or an error with `Error::kFailure` if variable or estimate not found.
   */
  Result Execute(Context_<T>& context, const ComputeBudget& budget) override {
    auto clock = middleware_->GetClockInterface();
    const auto current_time = clock->Now();

    // Check if we should broadcast: either first time (nullopt) or period elapsed.
    if (latest_broadcast_time_.has_value()) {
      const auto time_since_latest_broadcast = current_time - latest_broadcast_time_.value();
      if (broadcast_period_opt_ && (time_since_latest_broadcast < broadcast_period_opt_.value())) {
        return Ok();
      }
    }

    // Get the latest variable from the DBN
    auto variable_result = context.dbn.template GetLatestVariable<SE2Variable_<T>>(config_.variable_name);
    if (!variable_result) {
      middleware_->Error("SE2PoseBroadcaster: {}.", variable_result.error());
      return Err(CollectError(variable_result));
    }
    auto variable = Unwrap(variable_result);
    if (variable == nullptr) {
      middleware_->Error("SE2PoseBroadcaster: variable '{}' not found in DBN.", config_.variable_name);
      return Err(Error::kFailure, fmt::format("variable '{}' not found in DBN", config_.variable_name));
    }

    // Find the estimate
    Result_<std::shared_ptr<const OnManifoldEstimate_<T>>> estimate_result;
    if (!config_.target_estimate.empty()) {
      estimate_result = context.estimates.template GetLatest<const OnManifoldEstimate_<T>>(config_.target_estimate);
    } else {
      if (context.estimates.GetNumEstimateNames() > 1) {
        middleware_->Error("SE2PoseBroadcaster: estimate choice is ambiguous, specify one.");
        return Err(Error::kFailure, "estimate choice is ambiguous, specify one");
      }
      estimate_result = context.estimates.template GetLatestForVariable<const OnManifoldEstimate_<T>>(variable->Id());
    }
    if (!estimate_result) {
      middleware_->Error("SE2PoseBroadcaster: {}.", estimate_result.error());
      return Err(CollectError(estimate_result));
    }
    auto estimate = Unwrap(estimate_result);
    if (estimate == nullptr) {
      middleware_->Error("SE2PoseBroadcaster: no estimate found for variable '{}'.", config_.variable_name);
      return Err(Error::kFailure, fmt::format("no estimate found for variable '{}'", config_.variable_name));
    }

    auto distribution = variable->GetDistribution(*estimate);
    if (!distribution) {
      middleware_->Error("SE2PoseBroadcaster: Could not extract SE(2) distribution from estimate.");
      return Err(Error::kFailure, "could not extract SE(2) distribution from estimate");
    }

    // Compute the mean pose (origin ⊕ tangent_mean)
    beluga_estimation::SE2<T> pose;
    distribution->ComputeMean(pose);

    // Broadcast the transform
    auto transform_interface = middleware_->GetTransformInterface();
    transform_interface->BroadcastTransform(config_.target_frame, config_.source_frame, current_time, pose);

    middleware_->Debug(
        "SE2PoseBroadcaster: broadcast '{}' -> '{}' for variable '{}'", config_.source_frame, config_.target_frame,
        config_.variable_name);

    // Update the latest broadcast time
    latest_broadcast_time_ = current_time;

    return Ok();
  }

  /**
   * @brief Releases resources and performs cleanup.
   *
   * @param budget Resource limits for finalization.
   * @return Success.
   */
  Result Finalize(const ComputeBudget& budget) override { return Ok(); }

 private:
  std::shared_ptr<MiddlewareInterface> middleware_;
  Configuration config_;
  std::optional<std::chrono::nanoseconds> broadcast_period_opt_;
  std::optional<TimePoint> latest_broadcast_time_;
};

using SE2PoseBroadcaster = SE2PoseBroadcaster_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation::SE2PoseBroadcaster, beluga_estimation::BayesianOperation);
