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

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include <range/v3/view/sliding.hpp>
#include <range/v3/view/zip.hpp>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/probability.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/constraints/normal_constraint.hpp>
#include <beluga_estimation/constraints/se2_velocity_kinematic_constraint.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/core/schemas.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace beluga_estimation {

/**
 * @brief SE(2) kinematic state connector operation for Dynamic Bayesian Networks.
 *
 * The connector is a Bayesian operation designed to complete missing state information
 * in a Dynamic Bayesian Network (DBN) so that filtering algorithms (such as the Extended Kalman
 * Filter) receive a consistent, contiguous sequence of SE(2) Pose and 3D Euclidean Velocity states.
 *
 * The operation executes in three passes over the not-yet-processed slice of the
 * network's timeline:
 * 1. Iterates over existing Pose variables and adds missing Velocity variables at corresponding timestamps.
 * 2. Iterates over existing Velocity variables and adds missing Pose variables at corresponding timestamps.
 * 3. Zips the chronologically ordered Pose and Velocity sequences and, for every consecutive
 *    pair, adds a kinematic constraint (@c SE2VelocityKinematicConstraint) to the dependent
 *    Pose and a random-walk constraint (@c NormalConstraint) to the dependent Velocity,
 *    skipping either one when the target variable is already conditioned by some other
 *    constraint (e.g. an odometry process model or an IMU step constraint).
 *
 * The configured process noise covariances are continuous-time (per second). Each
 * constraint uses them scaled by the time step between its pair, so the process model
 * does not depend on how many variables the timeline holds.
 *
 * The operation is stateful and incremental: it remembers the latest timestamp it
 * completed and, on subsequent invocations, only looks at data from that timestamp
 * onwards. The lower bound is closed so that the pair spanning the boundary between
 * two invocations is still connected. Calling @ref Finalize resets that memory.
 *
 * Constraint names derive from the configured variable names so that several connector
 * instances can coexist in one DBN (e.g. for multi-agent estimation). The kinematic
 * constraint is named `<pose_variable_name>/kinematics` and the random-walk constraint
 * is named `<velocity_variable_name>/random_walk`.
 *
 * @tparam T Scalar type used by the filter (e.g., float or double).
 */
template <typename T>
class SE2KinematicStateConnector_ : public BayesianOperation_<T> {
 public:
  /**
   * @brief Configuration for the SE(2) kinematic state connector.
   */
  struct Configuration {
    /// Default isotropic process noise per second used by both the kinematic and random-walk factors.
    static constexpr double kDefaultProcessNoiseCovariance = 0.01;
    /// Default variable name used for SE2 pose states in the DBN.
    static constexpr std::string_view kDefaultPoseVariableName = kPoseVariableName;
    /// Default variable name used for Euclidean 3D velocity states in the DBN.
    static constexpr std::string_view kDefaultVelocityVariableName = kVelocityVariableName;

    /**
     * @brief Loads configuration from middleware parameters.
     * @param middleware The middleware interface.
     * @param prefix The parameter namespace prefix.
     * @return A populated Configuration instance.
     */
    static Configuration LoadFromParams(
        std::shared_ptr<MiddlewareInterface> middleware,
        const std::string& prefix = "") {
      Configuration config;
      config.pose_variable_name = middleware->DeclareParameter<std::string>(
          prefix + "pose_variable_name", std::string(kDefaultPoseVariableName));
      config.velocity_variable_name = middleware->DeclareParameter<std::string>(
          prefix + "velocity_variable_name", std::string(kDefaultVelocityVariableName));

      const auto default_covariance = std::vector<double>{kDefaultProcessNoiseCovariance, 0.0, 0.0, 0.0,
                                                          kDefaultProcessNoiseCovariance, 0.0, 0.0, 0.0,
                                                          kDefaultProcessNoiseCovariance};
      const auto load_covariance = [&middleware](const std::string& name, const std::vector<double>& default_value) {
        const auto values = middleware->DeclareParameter<std::vector<double>>(name, default_value);
        BELUGA_ESTIMATION_DEMAND(values.size() == 9, "parameter '{}' must have 9 elements (3x3 row-major)", name);
        return beluga_estimation::Matrix3<T>(
            beluga_estimation::Map<const beluga_estimation::Matrix3<T>>(values.data()));
      };
      config.pose_covariance = load_covariance(prefix + "pose_covariance", default_covariance);
      config.velocity_covariance = load_covariance(prefix + "velocity_covariance", default_covariance);
      return config;
    }

    std::string pose_variable_name{kDefaultPoseVariableName};
    std::string velocity_variable_name{kDefaultVelocityVariableName};
    /// Process noise per second of the kinematic constraint, over the SE(2) tangent [x, y, theta].
    beluga_estimation::Matrix3<T> pose_covariance{
        beluga_estimation::Matrix3<T>::Identity() * static_cast<T>(kDefaultProcessNoiseCovariance)};
    /// Covariance per second of the velocity random walk, over [vx, vy, omega].
    beluga_estimation::Matrix3<T> velocity_covariance{
        beluga_estimation::Matrix3<T>::Identity() * static_cast<T>(kDefaultProcessNoiseCovariance)};
  };

  /**
   * @brief Creates the connector from middleware parameters.
   * @param middleware The middleware interface.
   * @param prefix The parameter namespace prefix for this operation.
   * @return A result containing the connector instance.
   */
  static Result_<std::unique_ptr<SE2KinematicStateConnector_<T>>> Make(
      std::shared_ptr<MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<SE2KinematicStateConnector_<T>>(std::move(middleware), prefix);
  }

  /**
   * @brief Constructs the connector from middleware parameters.
   * @param middleware The middleware interface.
   * @param prefix The parameter namespace prefix.
   */
  explicit SE2KinematicStateConnector_(std::shared_ptr<MiddlewareInterface> middleware, std::string_view prefix = "")
      : SE2KinematicStateConnector_(middleware, Configuration::LoadFromParams(middleware, std::string(prefix))) {}

  /**
   * @brief Constructs the connector from an explicit configuration.
   * @param middleware The middleware interface.
   * @param config The connector configuration.
   */
  SE2KinematicStateConnector_(std::shared_ptr<MiddlewareInterface> middleware, Configuration config)
      : middleware_(std::move(middleware)), config_(std::move(config)) {
    BELUGA_ESTIMATION_DEMAND(
        IsCovarianceMatrix(config_.pose_covariance), "pose_covariance must be symmetric positive definite");
    BELUGA_ESTIMATION_DEMAND(
        IsCovarianceMatrix(config_.velocity_covariance), "velocity_covariance must be symmetric positive definite");
    // Derive constraint names from the configured variable names so that several
    // connectors can coexist in one DBN without colliding on (name, timestamp).
    kinematic_constraint_name_ = config_.pose_variable_name + "/kinematics";
    normal_constraint_name_ = config_.velocity_variable_name + "/random_walk";
  }

  /**
   * @brief Executes the connector operation on the context's DBN.
   *
   * Only the timeline slice starting at the latest timestamp completed by a
   * previous invocation is processed. An empty slice is a successful no-op.
   *
   * @param context Execution context with DBN and estimate store.
   * @param budget Unused.
   * @return Success, or an error with `Error::kFailure` if the DBN holds
   *         variables of the wrong type or dimension.
   */
  Result Execute(Context_<T>& context, const ComputeBudget& /*budget*/) override {
    // The lower bound is closed on purpose: the pair that spans the boundary
    // with the previous invocation must be re-examined so that the new data is
    // connected to what was already processed. Pairs that are already
    // conditioned are skipped by the HasConditioningConstraints checks below.
    const auto interval = latest_timestamp_ ? TimeInterval::From(*latest_timestamp_) : TimeInterval{};

    if (latest_timestamp_) {
      middleware_->Debug(
          "SE2KinematicStateConnector: processing timeline since timestamp {}",
          latest_timestamp_->time_since_epoch().count());
    } else {
      middleware_->Debug("SE2KinematicStateConnector: processing the full network timeline");
    }

    std::size_t num_velocity_variables_added = 0;
    std::size_t num_pose_variables_added = 0;

    if (auto status = CollectPoseIds(context, config_.pose_variable_name, interval, pose_ids_); !status) {
      middleware_->Error("SE2KinematicStateConnector: could not collect pose variables: {}.", status.error());
      return status;
    }
    if (auto status = CollectVelocityIds(context, config_.velocity_variable_name, interval, velocity_ids_); !status) {
      middleware_->Error("SE2KinematicStateConnector: could not collect velocity variables: {}.", status.error());
      return status;
    }
    middleware_->Debug(
        "SE2KinematicStateConnector: pass 1 found {} pose variable(s) and {} velocity variable(s)", pose_ids_.size(),
        velocity_ids_.size());

    // Pass 1: iterate over pose variables, adding missing velocity variables.
    auto velocity_ids_it = velocity_ids_.begin();
    for (const auto& pose_id : pose_ids_) {
      // Both sequences are sorted, so a velocity timestamp below the current
      // pose timestamp can no longer pair with a later pose either. Skipping it
      // keeps the insertion point, and therefore the order, correct.
      while (velocity_ids_it != velocity_ids_.end() && velocity_ids_it->timestamp < pose_id.timestamp) {
        ++velocity_ids_it;
      }
      if (velocity_ids_it != velocity_ids_.end() && velocity_ids_it->timestamp == pose_id.timestamp) {
        ++velocity_ids_it;
        continue;
      }
      auto velocity_id = Identifier{config_.velocity_variable_name, pose_id.timestamp};
      auto result = context.dbn.template Add<beluga_estimation::EuclideanVariable_<T>>(velocity_id, NDim<3>{});
      if (!result) {
        middleware_->Error(
            "SE2KinematicStateConnector: could not add velocity variable at timestamp {}.",
            pose_id.timestamp.time_since_epoch().count());
        return Err(CollectError(result));
      }
      velocity_ids_it = velocity_ids_.insert(velocity_ids_it, velocity_id);
      ++num_velocity_variables_added;
      middleware_->Debug(
          "SE2KinematicStateConnector: added missing '{}' variable at timestamp {}", config_.velocity_variable_name,
          pose_id.timestamp.time_since_epoch().count());
      ++velocity_ids_it;
    }

    // Pass 2: iterate over velocity variables, adding missing pose variables.
    middleware_->Debug(
        "SE2KinematicStateConnector: pass 2 found {} pose variable(s) and {} velocity variable(s)", pose_ids_.size(),
        velocity_ids_.size());
    auto pose_ids_it = pose_ids_.begin();
    for (const auto& velocity_id : velocity_ids_) {
      while (pose_ids_it != pose_ids_.end() && pose_ids_it->timestamp < velocity_id.timestamp) {
        ++pose_ids_it;
      }
      if (pose_ids_it != pose_ids_.end() && pose_ids_it->timestamp == velocity_id.timestamp) {
        ++pose_ids_it;
        continue;
      }
      auto pose_id = Identifier{config_.pose_variable_name, velocity_id.timestamp};
      auto result = context.dbn.template Add<beluga_estimation::SE2Variable_<T>>(pose_id);
      if (!result) {
        middleware_->Error(
            "SE2KinematicStateConnector: could not add pose variable at timestamp {}.",
            velocity_id.timestamp.time_since_epoch().count());
        return Err(CollectError(result));
      }
      pose_ids_it = pose_ids_.insert(pose_ids_it, pose_id);
      ++num_pose_variables_added;
      middleware_->Debug(
          "SE2KinematicStateConnector: added missing '{}' variable at timestamp {}", config_.pose_variable_name,
          velocity_id.timestamp.time_since_epoch().count());
      ++pose_ids_it;
    }
    BELUGA_ESTIMATION_ASSERT(pose_ids_.size() == velocity_ids_.size());

    std::size_t num_kinematic_constraints_added = 0;
    std::size_t num_normal_constraints_added = 0;

    // Pass 3: connect every consecutive pair that is not already conditioned.
    for (auto&& window : ranges::views::zip(pose_ids_, velocity_ids_) | ranges::views::sliding(2)) {
      const auto& [prev_pose_id, prev_velocity_id] = window[0];
      const auto& [next_pose_id, next_velocity_id] = window[1];
      BELUGA_ESTIMATION_ASSERT(prev_pose_id.timestamp == prev_velocity_id.timestamp);
      BELUGA_ESTIMATION_ASSERT(next_pose_id.timestamp == next_velocity_id.timestamp);
      // Timestamps are sorted and unique, so the step is positive.
      const T dt =
          std::chrono::duration_cast<std::chrono::duration<T>>(next_pose_id.timestamp - prev_pose_id.timestamp).count();

      if (!context.dbn.HasConditioningConstraints(next_pose_id)) {
        const Identifier constraint_id{kinematic_constraint_name_, next_pose_id.timestamp};
        auto added = context.dbn.template Add<beluga_estimation::SE2VelocityKinematicConstraint_<T>>(
            constraint_id, prev_pose_id, prev_velocity_id, next_pose_id, config_.pose_covariance * dt);
        // The target pose is unconditioned, so a failed insertion means the
        // identifier is taken by an unrelated constraint. Reporting the conflict
        // is required: skipping would leave the pose without a process model.
        if (!added) {
          middleware_->Error(
              "SE2KinematicStateConnector: could not add kinematic constraint at timestamp {}.",
              next_pose_id.timestamp.time_since_epoch().count());
          return Err(CollectError(added));
        }
        ++num_kinematic_constraints_added;
        middleware_->Debug(
            "SE2KinematicStateConnector: connected pose at timestamp {} to pose and velocity at timestamp {}",
            next_pose_id.timestamp.time_since_epoch().count(), prev_pose_id.timestamp.time_since_epoch().count());
      } else {
        middleware_->Debug(
            "SE2KinematicStateConnector: skipping kinematic constraint for pose at timestamp {} (already conditioned)",
            next_pose_id.timestamp.time_since_epoch().count());
      }

      if (!context.dbn.HasConditioningConstraints(next_velocity_id)) {
        const Identifier constraint_id{normal_constraint_name_, next_velocity_id.timestamp};
        auto added = context.dbn.template Add<beluga_estimation::NormalConstraint_<T, 3>>(
            constraint_id, prev_velocity_id, next_velocity_id,
            std::make_shared<const beluga_estimation::NormalDistribution_<T, 3>>(config_.velocity_covariance * dt));
        if (!added) {
          middleware_->Error(
              "SE2KinematicStateConnector: could not add velocity constraint at timestamp {}.",
              next_velocity_id.timestamp.time_since_epoch().count());
          return Err(CollectError(added));
        }
        ++num_normal_constraints_added;
        middleware_->Debug(
            "SE2KinematicStateConnector: connected velocity at timestamp {} to velocity at timestamp {}",
            next_velocity_id.timestamp.time_since_epoch().count(),
            prev_velocity_id.timestamp.time_since_epoch().count());
      } else {
        middleware_->Debug(
            "SE2KinematicStateConnector: skipping normal constraint for velocity at timestamp {} (already conditioned)",
            next_velocity_id.timestamp.time_since_epoch().count());
      }
    }

    if (num_velocity_variables_added + num_pose_variables_added > 0) {
      middleware_->Debug(
          "SE2KinematicStateConnector: added {} velocity variable(s) and {} pose variable(s)",
          num_velocity_variables_added, num_pose_variables_added);
    }
    if (num_kinematic_constraints_added + num_normal_constraints_added > 0) {
      middleware_->Debug(
          "SE2KinematicStateConnector: added {} kinematic constraint(s) and {} normal constraint(s)",
          num_kinematic_constraints_added, num_normal_constraints_added);
    }

    // Only advance once the whole slice was completed, so a failure is retried
    // on the next invocation instead of leaving a permanent hole.
    if (!pose_ids_.empty()) {
      latest_timestamp_ = pose_ids_.back().timestamp;
      middleware_->Debug(
          "SE2KinematicStateConnector: timeline completed up to timestamp {}",
          latest_timestamp_->time_since_epoch().count());
    }

    return Ok();
  }

  /**
   * @brief Discards the incremental state.
   *
   * The next call to @ref Execute reprocesses the whole network timeline.
   *
   * @param budget Unused.
   * @return Success.
   */
  Result Finalize(const ComputeBudget& /*budget*/) override {
    latest_timestamp_.reset();
    pose_ids_.clear();
    velocity_ids_.clear();
    return Ok();
  }

 private:
  /**
   * @brief Collects the identifiers of pose variables within a timeline slice.
   *
   * The identifiers are appended in chronological order, matching the network's
   * time-aware index. The output is cleared first, so the same buffer can be
   * reused across calls. The caller is responsible for reporting failures.
   *
   * @param context Execution context with the DBN to query.
   * @param variable_name Name of the pose variables to collect.
   * @param interval Timeline slice to collect identifiers from.
   * @param pose_ids Output buffer receiving the collected identifiers.
   * @return Success, or an error with `Error::kFailure` if a stored variable with
   *         the given name is not an SE(2) variable.
   */
  static Result CollectPoseIds(
      const Context_<T>& context,
      std::string_view variable_name,
      const TimeInterval& interval,
      std::vector<Identifier>& pose_ids) {
    pose_ids.clear();
    for (auto&& variable_result :
         context.dbn.template GetVariables<beluga_estimation::SE2Variable_<T>>(variable_name, interval)) {
      if (!variable_result) {
        return Err(CollectError(variable_result));
      }
      auto variable = Unwrap(variable_result);
      BELUGA_ESTIMATION_ASSERT(variable != nullptr);
      pose_ids.push_back(variable->Id());
    }
    return Ok();
  }

  /**
   * @brief Collects the identifiers of velocity variables within a timeline slice.
   *
   * The identifiers are appended in chronological order, matching the network's
   * time-aware index. The output is cleared first, so the same buffer can be
   * reused across calls. The caller is responsible for reporting failures.
   *
   * SE(2) kinematics are expressed in the 3D velocity tangent space, so velocity
   * variables of any other dimension are rejected here, before the constraints
   * that assume a 3D velocity are constructed.
   *
   * @param context Execution context with the DBN to query.
   * @param variable_name Name of the velocity variables to collect.
   * @param interval Timeline slice to collect identifiers from.
   * @param velocity_ids Output buffer receiving the collected identifiers.
   * @return Success, or an error with `Error::kFailure` if a stored variable with
   *         the given name is not a 3D Euclidean variable.
   */
  static Result CollectVelocityIds(
      const Context_<T>& context,
      std::string_view variable_name,
      const TimeInterval& interval,
      std::vector<Identifier>& velocity_ids) {
    velocity_ids.clear();
    for (auto&& variable_result :
         context.dbn.template GetVariables<beluga_estimation::EuclideanVariable_<T>>(variable_name, interval)) {
      if (!variable_result) {
        return Err(CollectError(variable_result));
      }
      auto variable = Unwrap(variable_result);
      BELUGA_ESTIMATION_ASSERT(variable != nullptr);
      if (variable->Size() != 3) {
        return Err(
            Error::kFailure,
            fmt::format(
                "velocity variable '{}' at timestamp {} has dimension {} but SE(2) kinematics require 3",
                variable->Id().name, variable->Id().timestamp.time_since_epoch().count(), variable->Size()));
      }
      velocity_ids.push_back(variable->Id());
    }
    return Ok();
  }

  std::shared_ptr<MiddlewareInterface> middleware_;
  Configuration config_;
  /// Cached kinematic constraint name, derived from the pose variable name.
  std::string kinematic_constraint_name_;
  /// Cached random-walk constraint name, derived from the velocity variable name.
  std::string normal_constraint_name_;
  /// Pose identifiers of the slice currently being processed (reused buffer).
  std::vector<Identifier> pose_ids_;
  /// Velocity identifiers of the slice currently being processed (reused buffer).
  std::vector<Identifier> velocity_ids_;
  /// Latest timestamp fully processed by a previous invocation, if any.
  std::optional<TimePoint> latest_timestamp_;
};

using SE2KinematicStateConnector = SE2KinematicStateConnector_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation::SE2KinematicStateConnector, beluga_estimation::BayesianOperation);
