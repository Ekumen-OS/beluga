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
 * @brief Bayesian operation for processing IMU measurements.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <fmt/format.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/algebra/projection.hpp>
#include <beluga_estimation/common/bitset.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/constraints/normal_likelihood_constraint.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/core/schemas.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/middleware/message_pipe.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/msg/imu.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>

namespace beluga_estimation {

/**
 * @brief Bayesian operation that processes IMU measurements and adds them to a DBN.
 *
 * This operation subscribes to IMU sensor messages and creates variables and likelihood constraints
 * in the Dynamic Bayesian Network representing angular velocity and/or linear acceleration
 * measurements.
 *
 * ## Features
 * - Configurable measurement enables (angular velocity, acceleration)
 * - 3D and 2D modes (planar robots)
 * - Frame transformations via TF
 * - Likelihood constraints for measurements
 *
 * ## Preconditions
 * - Linear acceleration measurements include the gravity vector. Applications
 *   requiring gravity-free acceleration must subtract it before this operation.
 * - The lever arm correction for linear acceleration assumes the body frame is
 *   rigidly attached to the robot (e.g. `base_link`) and the sensor frame is
 *   the IMU frame. Using a world frame (e.g. `map`) as the body frame makes
 *   the correction physically meaningless.
 *
 * ## Configuration Parameters
 * - `device_name`: Unique name for this IMU sensor (default: "imu")
 * - `topic_name`: Topic to subscribe to (default: "/imu/data")
 * - `body_frame`: Body frame onto which constraints apply (default: "" = sensor frame)
 * - `transform_timeout`: Timeout for TF lookups in seconds (default: 0.0)
 * - `velocity_variable_name`: Name of the velocity variable (default: "velocity")
 * - `acceleration_variable_name`: Name of the acceleration variable (default: "acceleration")
 * - `use_2d_mode`: Enable 2D planar mode (default: false)
 * - `angular_velocity_mask`: Boolean mask over `[wx, wy, wz]` selecting the
 *   angular velocity components to fuse (default: [true, true, true]; only `wz`
 *   may be enabled in 2D mode)
 * - `linear_acceleration_mask`: Boolean mask over `[ax, ay, az]` selecting the
 *   acceleration components to fuse (default: [false, false, false]; only `ax`
 *   and `ay` may be enabled in 2D mode)
 *
 * ## Example Usage
 * @code
 * auto middleware = std::make_shared<ROS2Middleware>(node);
 * auto imu_probe = std::make_shared<IMUProbe>(middleware);
 *
 * DynamicBayesianNetwork dbn;
 * imu_probe->Initialize(ComputeBudget::Unlimited());
 *
 * // In update loop:
 * Result status = imu_probe->Execute(dbn, budget);
 * if (status) {
 *   // Messages processed, variables/constraints added to dbn
 * }
 * @endcode
 *
 * ## Variable Naming
 * Variables follow the compact schema shared with other operations by default:
 * - "velocity" and "acceleration" (both R³ in 2D, R⁶ in 3D)
 *
 * Both names are configurable so the probe can feed a differently named state.
 * The gyroscope measurement constrains the angular slice of the "velocity"
 * variable (indices [2, 3) in 2D mode, [3, 6) in 3D mode), and the accelerometer
 * measurement constrains the linear slice of the "acceleration" variable
 * (indices [0, 2) in 2D mode, [0, 3) in 3D mode), so both fuse with other
 * sources of the same quantities. Each measurement distribution is projected
 * onto the masked components and the constrained coordinates are mapped onto
 * them through a projection matrix built once, at configuration time.
 *
 * Constraints are named `<device_name>/<variable_name>/measurement`, with the
 * configured variable name so that several probes can coexist in one DBN:
 * - "imu0/velocity/measurement", "imu1/acceleration/measurement"
 *
 * ## Nomenclature
 * Code variables follow the spatial quantity naming convention documented in
 * `docs/concepts/spatial-naming.md`. The frame glossary for this operation is:
 * - `B`: body frame (the `body_frame` parameter).
 * - `S`: sensor frame (the IMU message frame).
 * - `I`: inertial reference frame.
 *
 * The transform `X_BS` maps quantities expressed in the sensor frame S to the
 * body frame B. The gyroscope measurement is the angular velocity of the
 * sensor frame in the inertial frame, expressed in the sensor frame, `w_IS_S`;
 * the accelerometer measurement is the linear acceleration of the sensor point
 * in the inertial frame, expressed in the sensor frame, `a_IS_S`.
 *
 * @tparam T Scalar type (float or double)
 */
template <typename T>
class IMUProbe_ : public beluga_estimation::BayesianOperation_<T> {
 public:
  /**
   * @brief Configuration structure for IMU probe operation.
   */
  struct Configuration {
    /// @brief Load configuration from middleware parameters.
    /// @param middleware The middleware interface to load parameters from.
    /// @param prefix The prefix for parameter names.
    /// @return A populated configuration structure.
    static Configuration LoadFromParams(
        std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware,
        const std::string& prefix = "") {
      Configuration config;

      // Core settings
      config.device_name = middleware->DeclareParameter<std::string>(prefix + "device_name", "imu");
      config.topic_name = middleware->DeclareParameter<std::string>(prefix + "topic_name", "/imu/data");

      // Frame transformation settings
      config.body_frame = middleware->DeclareParameter<std::string>(prefix + "body_frame", "");
      config.transform_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(middleware->DeclareParameter<double>(prefix + "transform_timeout", 0.0)));

      // Variable names (compact schema defaults, shared across operations).
      config.velocity_variable_name = middleware->DeclareParameter<std::string>(
          prefix + "velocity_variable_name", std::string(kVelocityVariableName));
      config.acceleration_variable_name = middleware->DeclareParameter<std::string>(
          prefix + "acceleration_variable_name", std::string(kAccelerationVariableName));

      // Mode selection
      config.use_2d_mode = middleware->DeclareParameter<bool>(prefix + "use_2d_mode", false);

      // Measurement enables: boolean masks over the measured components, in
      // the style of robot_localization sensor configuration. The gyroscope
      // mask covers [wx, wy, wz] (only wz matters in 2D mode) and the
      // accelerometer mask covers [ax, ay, az] (only ax, ay matter in 2D mode).
      config.angular_velocity_mask = middleware->DeclareParameter<std::vector<bool>>(
          prefix + "angular_velocity_mask", std::vector<bool>{true, true, true});
      config.linear_acceleration_mask = middleware->DeclareParameter<std::vector<bool>>(
          prefix + "linear_acceleration_mask", std::vector<bool>{false, false, false});

      return config;
    }

    std::string device_name;                        ///< Unique identifier for this IMU sensor.
    std::string topic_name;                         ///< Topic to subscribe to for IMU messages.
    std::string body_frame;                         ///< Body frame onto which constraints apply (empty = sensor frame).
    std::chrono::nanoseconds transform_timeout{0};  ///< Timeout for TF lookups.
    std::string velocity_variable_name{kVelocityVariableName};          ///< Name of the velocity variable.
    std::string acceleration_variable_name{kAccelerationVariableName};  ///< Name of the acceleration variable.
    bool use_2d_mode = false;                                           ///< If true, use 2D measurements (planar).
    std::vector<bool> angular_velocity_mask{true, true, true};          ///< Whether to use [wx, wy, wz].
    std::vector<bool> linear_acceleration_mask{false, false, false};    ///< Whether to use [ax, ay, az].
  };

  /// @brief Fused velocity variable name for sensor fusion (shared across operations).
  static constexpr std::string_view kVelocityName = kVelocityVariableName;
  /// @brief Fused acceleration variable name for sensor fusion (shared across operations).
  static constexpr std::string_view kAccelerationName = kAccelerationVariableName;

  /// @brief Create an IMU probe with automatic parameter loading.
  /// @param middleware The middleware interface for communication and parameters.
  /// @param prefix The prefix for parameter names.
  /// @return A result containing the probe instance.
  static Result_<std::unique_ptr<IMUProbe_<T>>> Make(
      std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<IMUProbe_<T>>(std::move(middleware), prefix);
  }

  /// @brief Construct an IMU probe with automatic parameter loading.
  /// @param middleware The middleware interface for communication and parameters.
  explicit IMUProbe_(std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware, std::string_view prefix)
      : IMUProbe_(middleware, Configuration::LoadFromParams(middleware, std::string(prefix))) {}

  /// @brief Construct an IMU probe with explicit configuration.
  /// @param middleware The middleware interface for communication.
  /// @param config The configuration for this probe.
  IMUProbe_(std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware, Configuration config)
      : config_(std::move(config)), middleware_(std::move(middleware)), message_pipe_(middleware_, config_.topic_name) {
    BELUGA_ESTIMATION_DEMAND(
        config_.angular_velocity_mask.size() == 3, "angular_velocity_mask must have 3 entries [wx, wy, wz]");
    BELUGA_ESTIMATION_DEMAND(
        config_.linear_acceleration_mask.size() == 3, "linear_acceleration_mask must have 3 entries [ax, ay, az]");
    // Map the measured components onto the constrained variable coordinates and
    // precompute constraint names, derived from the configured variable names,
    // based on the enabled elements.
    const auto variable_dimension = static_cast<Eigen::Index>(config_.use_2d_mode ? 3 : 6);
    std::vector<bool> velocity_mask(static_cast<std::size_t>(variable_dimension), false);
    std::vector<bool> acceleration_mask(static_cast<std::size_t>(variable_dimension), false);
    if (config_.use_2d_mode) {
      // 2D mode: only the yaw rate (omega_z) is meaningful.
      if (config_.linear_acceleration_mask[2]) {
        middleware_->Warn(
            "IMUProbe: linear_acceleration along z-axis is not applicable in 2D mode; ignoring it for '{}'",
            config_.device_name);
      }
      acceleration_mask[0] = config_.linear_acceleration_mask[0];
      acceleration_mask[1] = config_.linear_acceleration_mask[1];
      if (config_.angular_velocity_mask[0] || config_.angular_velocity_mask[1]) {
        middleware_->Warn(
            "IMUProbe: angular_velocity around x or y-axis is not applicable in 2D mode; ignoring them for '{}'",
            config_.device_name);
      }
      velocity_mask[2] = config_.angular_velocity_mask[2];
    } else {
      // 3D mode: the angular coordinates are the last three of the 6D variable.
      for (std::size_t i = 0; i < 3; ++i) {
        acceleration_mask[i] = config_.linear_acceleration_mask[i];
        velocity_mask[i + 3] = config_.angular_velocity_mask[i];
      }
    }
    auto make_constraint_name = [this](const std::string& variable_name) {
      return config_.device_name + "/" + variable_name + "/measurement";
    };
    use_angular_velocity_ = AnyBitSet(velocity_mask);
    if (use_angular_velocity_) {
      angular_velocity_projection_ = MakeProjectionMatrix<T>(velocity_mask);
      angular_velocity_constraint_name_ = make_constraint_name(config_.velocity_variable_name);
    }
    use_linear_acceleration_ = AnyBitSet(acceleration_mask);
    if (use_linear_acceleration_) {
      acceleration_projection_ = MakeProjectionMatrix<T>(acceleration_mask);
      acceleration_constraint_name_ = make_constraint_name(config_.acceleration_variable_name);
    }
  }

  /// @brief Initialize the operation (open the message pipe).
  /// @param budget Resource limits for initialization.
  /// @return Success, or an error if the subscription could not be created.
  Result Initialize(const ComputeBudget& /*budget*/) override { return message_pipe_.Open(); }

  /// @brief Finalize the operation (close the message pipe).
  /// @param budget Resource limits for finalization.
  /// @return Success.
  Result Finalize(const ComputeBudget& /*budget*/) override {
    message_pipe_.Close();
    return Ok();
  }

  /// @brief Execute the operation on the DBN (process queued IMU messages).
  /// @param context The computation context with DBN and estimate store.
  /// @param budget Resource limits for this invocation.
  /// @return Success if messages were processed, an error with `Error::kBusy` if no messages were processed or a
  ///         required transform was unavailable, or an error otherwise.
  Result Execute(Context_<T>& context, const ComputeBudget& budget) override {
    std::size_t num_processed_messages = 0;
    while (auto message = message_pipe_.Fetch()) {
      const auto timestamp = message->GetTimestamp();

      // Single TF lookup per dequeued message (shared across processors).
      // X_BS is the pose of the sensor frame S in the body frame B; it
      // maps quantities expressed in S to B.
      std::optional<beluga_estimation::SE3d> X_BS = std::nullopt;
      if (!config_.body_frame.empty() && (use_angular_velocity_ || use_linear_acceleration_)) {
        const auto sensor_frame_id = message->GetFrameId();
        X_BS = middleware_->GetTransformInterface()->LookupTransform(
            config_.body_frame, sensor_frame_id, timestamp,
            std::optional<std::chrono::nanoseconds>(config_.transform_timeout));
        if (!X_BS) {
          middleware_->Warn(
              "IMUProbe: could not look up transform from '{}' to '{}' for '{}' device", sensor_frame_id,
              config_.body_frame, config_.device_name);
          return Err(Error::kBusy, "failed to look up sensor transform");
        }
      }

      Result status;

      if (use_angular_velocity_) {
        status = ProcessAngularVelocity(context, message, X_BS);
      }

      if (status && use_linear_acceleration_) {
        status = ProcessLinearAcceleration(context, message, X_BS);
      }

      if (!status) {
        middleware_->Error(
            "IMUProbe: failed to process message from '{}' device at timestamp {}", config_.device_name,
            timestamp.time_since_epoch().count());
        return status;
      }

      status = budget.Check(middleware_->GetClockInterface()->Now());
      if (!status) {
        middleware_->Debug(
            "IMUProbe: processed {} message(s) from '{}' device before the budget expired", num_processed_messages,
            config_.device_name);
        return status;
      }

      ++num_processed_messages;
    }

    middleware_->Debug(
        "IMUProbe: processed {} message(s) from '{}' device", num_processed_messages, config_.device_name);

    return num_processed_messages > 0 ? Ok() : Err(Error::kBusy);
  }

 private:
  Result_<std::shared_ptr<const EuclideanVariable_<T>>> EnsureEuclideanVariable(
      Context_<T>& context,
      const Identifier& variable_id) {
    if (config_.use_2d_mode) {
      auto candidate = context.dbn.template MaybeAdd<EuclideanVariable_<T>>(variable_id, NDim<3>{});
      if (!candidate) {
        return Err(CollectError(candidate));
      }
      auto variable = Unwrap(candidate);
      BELUGA_ESTIMATION_ASSERT(variable != nullptr);
      if (variable->Size() != 3) {
        auto error = Err(fmt::format(
            "variable '{}' has dimension {} but the IMU probe requires 3", variable_id.name, variable->Size()));
        middleware_->Error("IMUProbe: {}.", error.value());
        return error;
      }
      return variable;
    } else {
      auto candidate = context.dbn.template MaybeAdd<EuclideanVariable_<T>>(variable_id, NDim<6>{});
      if (!candidate) {
        return Err(CollectError(candidate));
      }
      auto variable = Unwrap(candidate);
      BELUGA_ESTIMATION_ASSERT(variable != nullptr);
      if (variable->Size() != 6) {
        auto error = Err(fmt::format(
            "variable '{}' has dimension {} but the IMU probe requires 6", variable_id.name, variable->Size()));
        middleware_->Error("IMUProbe: {}.", error.value());
        return error;
      }
      return variable;
    }
  }

  /// @brief Process angular velocity measurement from IMU message.
  /// @param context The computation context.
  /// @param message The IMU message containing the measurement.
  /// @param X_BS Optional pose of the sensor frame S in the body frame B.
  /// When provided, the measurement is re-expressed from the sensor frame to
  /// the body frame.
  /// @return Success, or an error with `Error::kFailure`.
  Result ProcessAngularVelocity(
      Context_<T>& context,
      const std::shared_ptr<const beluga_estimation::msg::Imu>& message,
      const std::optional<beluga_estimation::SE3d>& X_BS) {
    const auto timestamp = message->GetTimestamp();
    // Gyroscope measurement: angular velocity of the sensor frame S in
    // the (fixed, distant) inertial reference frame I, expressed in the
    // sensor frame S.
    const auto Mw_IS_S = message->GetAngularVelocity();
    const auto Sww_IS_S = message->GetAngularVelocityCovariance();

    beluga_estimation::Vector3d Mw_IB_B;
    beluga_estimation::Matrix3d Sww_IB_B;
    if (X_BS) {
      const auto R_BS = X_BS->rotationMatrix();
      Mw_IB_B = R_BS * Mw_IS_S;
      Sww_IB_B = R_BS * Sww_IS_S * R_BS.transpose();
    } else {
      // No transform, the body frame B is the sensor frame S.
      Mw_IB_B = Mw_IS_S;
      Sww_IB_B = Sww_IS_S;
    }

    // Create identifiers — variable name is shared across probes (sensor fusion),
    // constraint name is precomputed at construction from the configured names.
    Identifier variable_id{config_.velocity_variable_name, timestamp};

    auto variable_result = EnsureEuclideanVariable(context, variable_id);
    if (!variable_result) {
      middleware_->Error("IMUProbe: {}", variable_result.error());
      return Err(CollectError(variable_result));
    }
    auto variable = Unwrap(variable_result);
    BELUGA_ESTIMATION_ASSERT(variable != nullptr);
    middleware_->Debug(
        "IMUProbe: added angular velocity variable '{}' at timestamp {} from '{}' device", variable_id.name,
        variable_id.timestamp.time_since_epoch().count(), config_.device_name);

    // Project the gyroscope distribution onto the fused components. For
    // Gaussian measurements this is an exact marginalization.
    auto measurement_distribution = std::make_shared<const NormalDistribution_<T>>(
        Project(Mw_IB_B.template cast<T>(), config_.angular_velocity_mask),
        Project(Sww_IB_B.template cast<T>(), config_.angular_velocity_mask));
    auto constraint = context.dbn.template Add<NormalLikelihoodConstraint_<T>>(
        Identifier{angular_velocity_constraint_name_, timestamp}, variable->Id(), std::move(measurement_distribution),
        angular_velocity_projection_);
    if (!constraint) {
      middleware_->Error("IMUProbe: {}", constraint.error());
      return Err(CollectError(constraint));
    }
    middleware_->Debug(
        "IMUProbe: added angular velocity constraint '{}' at timestamp {} from '{}' device",
        angular_velocity_constraint_name_, timestamp.time_since_epoch().count(), config_.device_name);

    return Ok();
  }

  /// @brief Process linear acceleration measurement from IMU message.
  /// @param context The computation context.
  /// @param message The IMU message containing the measurement.
  /// @param X_BS Optional pose of the sensor frame S in the body frame B.
  /// When provided, the measurement is lever-arm corrected and re-expressed
  /// from the sensor frame to the body frame.
  /// @return Success, or an error with `Error::kFailure`.
  Result ProcessLinearAcceleration(
      Context_<T>& context,
      const std::shared_ptr<const beluga_estimation::msg::Imu>& message,
      const std::optional<beluga_estimation::SE3d>& X_BS) {
    const auto timestamp = message->GetTimestamp();
    // Accelerometer measurement: linear acceleration of the sensor frame S in
    // a (local free-falling) inertial reference frame I, expressed in the
    // sensor frame S.
    const auto Ma_IS_S = message->GetLinearAcceleration();
    const auto Saa_IS_S = message->GetLinearAccelerationCovariance();

    beluga_estimation::Vector3d Ma_IB_B;
    beluga_estimation::Matrix3d Saa_IB_B;
    if (X_BS) {
      // First order approximation of linear acceleration transformation,
      // ignoring angular acceleration (which would require separate estimation).
      //
      // Transforming the acceleration measurement expressed in the sensor
      // frame S to the body frame B origin subtracts the centripetal
      // acceleration term w x (w x r), where r = -R^T t is the lever arm
      // (sensor offset) expressed in the sensor frame S and w is the angular
      // velocity (also expressed in the sensor frame S). The covariance is
      // propagated through the Jacobian of the centripetal acceleration term
      // with respect to angular velocity.
      const auto R_BS = X_BS->rotationMatrix();
      const auto p_BS = X_BS->translation();
      const auto Mw_IS_S = message->GetAngularVelocity();
      const auto Sww_IS_S = message->GetAngularVelocityCovariance();
      const auto p_SB = -R_BS.transpose() * p_BS;
      const auto w_cross = ToCrossProductMatrix(Mw_IS_S);
      const auto r_cross = ToCrossProductMatrix(p_SB);
      const auto Mca_IS_S = Mw_IS_S.cross(Mw_IS_S.cross(p_SB));
      const auto Jw_ca_IS_S = r_cross * w_cross - 2 * w_cross * r_cross;
      Ma_IB_B = R_BS * (Ma_IS_S + Mca_IS_S);
      Saa_IB_B = R_BS * (Saa_IS_S + Jw_ca_IS_S * Sww_IS_S * Jw_ca_IS_S.transpose()) * R_BS.transpose();
    } else {
      // No transform, the body frame B is the sensor frame S.
      Ma_IB_B = Ma_IS_S;
      Saa_IB_B = Saa_IS_S;
    }

    // Create identifiers — variable name is shared across probes (sensor fusion),
    // constraint name is precomputed at construction from the configured names.
    Identifier variable_id{config_.acceleration_variable_name, timestamp};

    auto variable_result = EnsureEuclideanVariable(context, variable_id);
    if (!variable_result) {
      middleware_->Error("IMUProbe: {}", variable_result.error());
      return Err(CollectError(variable_result));
    }
    auto variable = Unwrap(variable_result);
    BELUGA_ESTIMATION_ASSERT(variable != nullptr);
    middleware_->Debug(
        "IMUProbe: added acceleration variable '{}' at timestamp {} from '{}' device", variable_id.name,
        variable_id.timestamp.time_since_epoch().count(), config_.device_name);

    // Project the accelerometer distribution onto the fused components. For
    // Gaussian measurements this is an exact marginalization.
    auto measurement_distribution = std::make_shared<const NormalDistribution_<T>>(
        Project(Ma_IB_B.template cast<T>(), config_.linear_acceleration_mask),
        Project(Saa_IB_B.template cast<T>(), config_.linear_acceleration_mask));
    auto constraint = context.dbn.template Add<NormalLikelihoodConstraint_<T>>(
        Identifier{acceleration_constraint_name_, timestamp}, variable->Id(), std::move(measurement_distribution),
        acceleration_projection_);
    if (!constraint) {
      middleware_->Error("IMUProbe: {}", constraint.error());
      return Err(CollectError(constraint));
    }
    middleware_->Debug(
        "IMUProbe: added acceleration constraint '{}' at timestamp {} from '{}' device", acceleration_constraint_name_,
        timestamp.time_since_epoch().count(), config_.device_name);

    return Ok();
  }

  Configuration config_;
  std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware_;
  MessagePipe<beluga_estimation::msg::Imu> message_pipe_;
  bool use_angular_velocity_ = false;
  bool use_linear_acceleration_ = false;
  std::string angular_velocity_constraint_name_;
  std::string acceleration_constraint_name_;
  beluga_estimation::MatrixX<T> angular_velocity_projection_;
  beluga_estimation::MatrixX<T> acceleration_projection_;
};

using IMUProbe = IMUProbe_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation::IMUProbe, beluga_estimation::BayesianOperation);
