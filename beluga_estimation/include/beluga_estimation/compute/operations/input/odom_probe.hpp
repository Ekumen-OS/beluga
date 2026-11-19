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
 * @brief Bayesian operation for processing odometry measurements.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <fmt/format.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/algebra/projection.hpp>
#include <beluga_estimation/common/bitset.hpp>
#include <beluga_estimation/common/probability.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/constraints/normal_likelihood_constraint.hpp>
#include <beluga_estimation/constraints/se2_normal_pose_likelihood_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/core/schemas.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>
#include <beluga_estimation/middleware/message_pipe.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/msg/odometry.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace beluga_estimation {

/**
 * @brief Bayesian operation that processes odometry measurements and adds them to a DBN.
 *
 * This operation subscribes to odometry messages and creates variables and
 * likelihood constraints in the Dynamic Bayesian Network representing the
 * measured pose and/or body-frame velocities, in 2D (planar) mode.
 *
 * ## Features
 * - Boolean mask selecting the measured velocity components (robot_localization style)
 * - Configurable variable names for sensor fusion across schemas
 * - 2D (planar) measurements only
 * - Frame transformations via TF
 * - Likelihood constraints for measurements
 *
 * ## Preconditions
 * - Odometry pose and twist are expressed in the message reference and body
 *   frames respectively, as per the standard ROS odometry convention.
 *
 * ## Configuration Parameters
 * - `device_name`: Unique name for this odometry source (default: "odom")
 * - `topic_name`: Topic to subscribe to (default: "odom")
 * - `world_frame`: Frame in which the pose measurement is expressed after
 *   transformation (default: "" = no transform)
 * - `base_frame`: Frame in which the twist measurement is expressed after
 *   transformation (default: "" = no transform)
 * - `transform_timeout`: Timeout for TF lookups in seconds (default: 0.0)
 * - `pose_variable_name`: Name of the pose variable (default: "pose")
 * - `velocity_variable_name`: Name of the velocity variable (default: "velocity")
 * - `use_pose`: Process the pose measurement (default: true)
 * - `twist_mask`: Boolean mask over the reported twist components
 *   `[vx, vy, omega]` selecting the ones to fuse (default: [true, true, true])
 *
 * ## Example Usage
 * @code
 * auto middleware = std::make_shared<ROS2Middleware>(node);
 * auto odom_probe = std::make_shared<OdomProbe>(middleware);
 *
 * DynamicBayesianNetwork dbn;
 * odom_probe->Initialize(ComputeBudget::Unlimited());
 *
 * // In update loop:
 * Result status = odom_probe->Execute(dbn, budget);
 * if (status) {
 *   // Messages processed, variables/constraints added to dbn
 * }
 * @endcode
 *
 * ## Variable Naming
 * Variables follow the compact schema shared with other operations by default:
 * - "pose" (SE(2)) and "velocity" (R³, body-frame `[vx, vy, omega]`)
 *
 * Both names are configurable so the probe can feed a differently named state.
 * The velocity mask selects the components of the velocity measurement that are
 * fused, in the same spirit as robot_localization sensor configuration arrays.
 * The probe projects the twist distribution onto those components and hands the
 * result to a likelihood constraint together with a projection matrix that
 * maps the constrained velocity coordinates onto them. Pose measurements always
 * observe the whole pose; partial pose observations are deferred until noise
 * Jacobians are available. Note the "velocity" name is shared with @ref IMUProbe_
 * for fusion purposes: in 2D mode the IMU probe constrains the angular
 * coordinate of the same variable.
 *
 * Constraints are named `<device_name>/<variable_name>/measurement`, with the
 * configured variable name so that several probes can coexist in one DBN:
 * - "odom0/pose/measurement", "odom1/velocity/measurement"
 *
 * ## Nomenclature
 * Code variables follow the spatial quantity naming convention documented in
 * `docs/concepts/spatial-naming.md`. The frame glossary for this operation is:
 * - `W`: world frame (the `world_frame` parameter).
 * - `O`: odometry frame (the message reference frame).
 * - `B`: base frame (the `base_frame` parameter).
 * - `Bo`: body frame (the message body frame).
 *
 * The pose measurement is the pose of the body frame Bo in the odometry frame
 * O, `X_OBo`; the twist measurement is the spatial velocity of the body frame
 * in the world frame W (assumed inertial), expressed in the body frame Bo.
 *
 * @tparam T Scalar type (float or double)
 */
template <typename T>
class OdomProbe_ : public BayesianOperation_<T> {
 public:
  /**
   * @brief Configuration structure for odometry probe operation.
   */
  struct Configuration {
    /// @brief Load configuration from middleware parameters.
    /// @param middleware The middleware interface to load parameters from.
    /// @param prefix The prefix for parameter names.
    /// @return A populated configuration structure.
    static Configuration LoadFromParams(
        std::shared_ptr<MiddlewareInterface> middleware,
        const std::string& prefix = "") {
      Configuration config;

      // Core settings
      config.device_name = middleware->DeclareParameter<std::string>(prefix + "device_name", "odom");
      config.topic_name = middleware->DeclareParameter<std::string>(prefix + "topic_name", "odom");

      // Frame transformation settings
      config.world_frame = middleware->DeclareParameter<std::string>(prefix + "world_frame", "");
      config.base_frame = middleware->DeclareParameter<std::string>(prefix + "base_frame", "");
      config.transform_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(middleware->DeclareParameter<double>(prefix + "transform_timeout", 0.0)));

      // Variable names (compact schema defaults, shared across operations).
      config.pose_variable_name =
          middleware->DeclareParameter<std::string>(prefix + "pose_variable_name", std::string(kPoseVariableName));
      config.velocity_variable_name = middleware->DeclareParameter<std::string>(
          prefix + "velocity_variable_name", std::string(kVelocityVariableName));

      // Measurement enables: the pose is observed as a whole; the twist is fused
      // component-wise via a boolean mask, robot_localization style.
      config.use_pose = middleware->DeclareParameter<bool>(prefix + "use_pose", true);
      config.twist_mask =
          middleware->DeclareParameter<std::vector<bool>>(prefix + "twist_mask", std::vector<bool>{true, true, true});

      return config;
    }

    std::string device_name;                        ///< Unique identifier for this odometry source.
    std::string topic_name;                         ///< Topic to subscribe to.
    std::string world_frame;                        ///< Frame in which the pose is expressed (empty = no transform).
    std::string base_frame;                         ///< Frame in which the twist is expressed (empty = no transform).
    std::chrono::nanoseconds transform_timeout{0};  ///< Timeout for TF lookups.
    std::string pose_variable_name{kPoseVariableName};          ///< Name of the pose variable.
    std::string velocity_variable_name{kVelocityVariableName};  ///< Name of the velocity variable.
    bool use_pose = true;                                       ///< Whether to process the pose measurement.
    std::vector<bool> twist_mask{true, true, true};             ///< Whether to use [vx, vy, omega].
  };

  /// @brief Pose variable name for sensor fusion (shared across operations).
  static constexpr std::string_view kPoseName = kPoseVariableName;
  /// @brief Fused velocity variable name for sensor fusion (shared across operations).
  static constexpr std::string_view kVelocityName = kVelocityVariableName;

  /// @brief Create an odometry probe with automatic parameter loading.
  /// @param middleware The middleware interface for communication and parameters.
  /// @param prefix The prefix for parameter names.
  /// @return A result containing the probe instance.
  static Result_<std::unique_ptr<OdomProbe_<T>>> Make(
      std::shared_ptr<MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<OdomProbe_<T>>(std::move(middleware), prefix);
  }

  /// @brief Construct an odometry probe with automatic parameter loading.
  /// @param middleware The middleware interface for communication and parameters.
  /// @param prefix The prefix for parameter names.
  explicit OdomProbe_(std::shared_ptr<MiddlewareInterface> middleware, std::string_view prefix)
      : OdomProbe_(middleware, Configuration::LoadFromParams(middleware, std::string(prefix))) {}

  /// @brief Construct an odometry probe with explicit configuration.
  /// @param middleware The middleware interface for communication.
  /// @param config The configuration for this probe.
  OdomProbe_(std::shared_ptr<MiddlewareInterface> middleware, Configuration config)
      : config_(std::move(config)), middleware_(std::move(middleware)), message_pipe_(middleware_, config_.topic_name) {
    BELUGA_ESTIMATION_DEMAND(config_.twist_mask.size() == 3, "twist_mask must have 3 entries [vx, vy, omega]");
    // Precompute constraint names once, derived from the configured variable names, so
    // that several probes can coexist in one DBN without colliding on (name, timestamp).
    auto make_constraint_name = [this](const std::string& variable_name) {
      return config_.device_name + "/" + variable_name + "/measurement";
    };
    use_pose_ = config_.use_pose;
    if (use_pose_) {
      pose_constraint_name_ = make_constraint_name(config_.pose_variable_name);
    }
    use_twist_ = AnyBitSet(config_.twist_mask);
    if (use_twist_) {
      // The fused twist components constrain the velocity coordinates the mask
      // selects; in the compact schema the twist ordering matches the variable's.
      twist_projection_ = MakeProjectionMatrix<T>(config_.twist_mask);
      twist_constraint_name_ = make_constraint_name(config_.velocity_variable_name);
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

  /// @brief Execute the operation on the DBN (process queued odometry messages).
  /// @param context The computation context with DBN and estimate store.
  /// @param budget Resource limits for this invocation.
  /// @return Success if messages were processed, an error with `Error::kBusy` if no messages were processed or a
  ///         required transform was unavailable, or an error otherwise.
  Result Execute(Context_<T>& context, const ComputeBudget& budget) override {
    std::size_t num_processed_messages = 0;

    while (auto message = message_pipe_.Fetch()) {
      const auto timestamp = message->GetTimestamp();

      // Resolve frame transforms up front, before touching the DBN, so that a
      // lookup failure leaves the network untouched and no downstream element
      // is processed on behalf of a failed message.
      // X_WO is the pose of the odometry frame O in the world frame W; it maps
      // quantities expressed in O to W.
      std::optional<beluga_estimation::SE3d> X_WO;
      if (!config_.world_frame.empty() && config_.use_pose) {
        const auto odom_frame_id = message->GetFrameId();
        X_WO = middleware_->GetTransformInterface()->LookupTransform(
            config_.world_frame, odom_frame_id, timestamp,
            std::optional<std::chrono::nanoseconds>(config_.transform_timeout));
        if (!X_WO) {
          middleware_->Warn(
              "OdomProbe: could not look up transform from '{}' to '{}' for '{}' device", odom_frame_id,
              config_.world_frame, config_.device_name);
          return Err(Error::kBusy, "failed to look up odometry transform");
        }
      }

      // X_BBo is the pose of the body frame Bo in the base frame B; it maps
      // quantities expressed in Bo to B.
      std::optional<beluga_estimation::SE3d> X_BBo;
      if (!config_.base_frame.empty() && (use_pose_ || use_twist_)) {
        const auto body_frame_id = message->GetBodyFrameId();
        X_BBo = middleware_->GetTransformInterface()->LookupTransform(
            config_.base_frame, body_frame_id, timestamp,
            std::optional<std::chrono::nanoseconds>(config_.transform_timeout));
        if (!X_BBo) {
          middleware_->Warn(
              "OdomProbe: could not look up transform from '{}' to '{}' for '{}' device", body_frame_id,
              config_.base_frame, config_.device_name);
          return Err(Error::kBusy, "failed to look up body transform");
        }
      }

      Result status;
      if (use_pose_) {
        status = ProcessPose(context, message, X_WO, X_BBo);
      }
      if (status && use_twist_) {
        status = ProcessTwist(context, message, X_BBo);
      }

      if (!status) {
        middleware_->Error(
            "OdomProbe: failed to process message from '{}' device at timestamp {}", config_.device_name,
            timestamp.time_since_epoch().count());
        return status;
      }

      status = budget.Check(middleware_->GetClockInterface()->Now());
      if (!status) {
        middleware_->Debug(
            "OdomProbe: processed {} message(s) from '{}' device before the budget expired", num_processed_messages,
            config_.device_name);
        return status;
      }
      ++num_processed_messages;
    }

    middleware_->Debug(
        "OdomProbe: processed {} message(s) from '{}' device", num_processed_messages, config_.device_name);

    return num_processed_messages > 0 ? Ok() : Err(Error::kBusy);
  }

 private:
  /// @brief Process the pose measurement from an odometry message.
  ///
  /// The pose measurement constrains the base frame B in the world frame W.
  /// By default, the world frame W is the odometry frame O, and the base frame
  /// B is the body frame Bo, unless specific transforms @p X_WO or @p X_BBo
  /// are provided.
  ///
  /// @param context The computation context.
  /// @param message The odometry message containing the measurement.
  /// @param X_WO Optional pose of the odometry frame O in the world frame W.
  /// @param X_BBo Optional pose of the body frame Bo in the base frame B.
  /// @return Success, or an error with `Error::kFailure`.
  Result ProcessPose(
      Context_<T>& context,
      const std::shared_ptr<const beluga_estimation::msg::Odometry>& message,
      const std::optional<beluga_estimation::SE3d>& X_WO,
      const std::optional<beluga_estimation::SE3d>& X_BBo) {
    const auto timestamp = message->GetTimestamp();

    // Pose measurement: pose of the body frame Bo in the odometry frame O.
    const auto Mx_OBo = message->GetPose();
    const auto Sxx_OBo = message->GetPoseCovariance();

    beluga_estimation::SE3d Mx_WBo;
    if (X_WO) {
      Mx_WBo = *X_WO * Mx_OBo;
    } else {
      // No transform, the world frame W is the odometry frame O.
      Mx_WBo = Mx_OBo;
    }
    // The right-trivialized (body-frame) covariance is invariant under left
    // composition with a fixed transform.
    beluga_estimation::Matrix6d Sxx_WBo = Sxx_OBo;

    beluga_estimation::SE3d Mx_WB;
    beluga_estimation::Matrix6d Sxx_WB;
    if (X_BBo) {
      const auto X_BoB = X_BBo->inverse();
      Mx_WB = Mx_WBo * X_BoB;
      // Right composition with X_BoB = X_BBo⁻¹ transforms
      // the right-trivialized covariance with the Adjoint of X_BBo.
      const auto Ad_X_BBo = X_BBo->Adj();
      Sxx_WB = Ad_X_BBo * Sxx_WBo * Ad_X_BBo.transpose();
    } else {
      // No transform, the base frame B is the body frame Bo.
      Mx_WB = Mx_WBo;
      Sxx_WB = Sxx_WBo;
    }

    // Create identifiers — variable name is shared across probes (sensor fusion),
    // constraint name is precomputed at construction from the configured names.
    Identifier variable_id{config_.pose_variable_name, timestamp};

    const beluga_estimation::SE2<T> Mx_2d_WB = ToSE2(Mx_WB.template cast<T>());
    const beluga_estimation::Matrix3<T> Sxx_2d_WB = ToSE2Covariance(Sxx_WB.template cast<T>());
    auto candidate = context.dbn.template MaybeAdd<SE2Variable_<T>>(variable_id);
    if (!candidate) {
      return Err(CollectError(candidate));
    }
    auto variable = Unwrap(candidate);
    BELUGA_ESTIMATION_ASSERT(variable != nullptr);

    middleware_->Debug(
        "OdomProbe: added pose variable '{}' at timestamp {} from '{}' device", variable_id.name,
        variable_id.timestamp.time_since_epoch().count(), config_.device_name);

    auto measurement_distribution = std::make_shared<SE2NormalDistribution_<T>>(Mx_2d_WB, Sxx_2d_WB);
    auto constraint = context.dbn.template Add<SE2NormalPoseLikelihoodConstraint_<T>>(
        Identifier{pose_constraint_name_, timestamp}, *measurement_distribution, variable->Id());

    if (!constraint) {
      middleware_->Warn("Failed to add pose constraint");
      return Err(CollectError(constraint));
    }
    middleware_->Debug(
        "OdomProbe: added pose constraint '{}' at timestamp {} from '{}' device", pose_constraint_name_,
        timestamp.time_since_epoch().count(), config_.device_name);

    return Ok();
  }

  /// @brief Process the twist measurement from an odometry message.
  ///
  /// The twist measurement is the spatial velocity of the body frame Bo in the
  /// world frame W (assumed inertial), expressed in the body frame Bo. The
  /// enabled channels select the slice of the planar velocity `[vx, vy, omega]`
  /// that the measurement constrains: `[0, 2)` for linear velocity, `[2, 3)`
  /// for angular velocity, or the whole vector when both are enabled.
  ///
  /// @param context The computation context.
  /// @param message The odometry message containing the measurement.
  /// @param X_BBo Optional pose of the body frame Bo in the base frame B.
  /// When provided, each quantity is re-expressed from the body frame to the
  /// base frame.
  /// @return Success, or an error with `Error::kFailure`.
  Result ProcessTwist(
      Context_<T>& context,
      const std::shared_ptr<const beluga_estimation::msg::Odometry>& message,
      const std::optional<beluga_estimation::SE3d>& X_BBo) {
    const auto timestamp = message->GetTimestamp();

    // Keep the message views alive in locals; the blocks below are lazy
    // expressions referencing them.
    const auto Mt_WBo_Bo = message->GetTwist();
    const auto Stt_WBo_Bo = message->GetTwistCovariance();
    const auto Mv_WBo_Bo = Mt_WBo_Bo.head<3>();
    const auto Mw_WBo_Bo = Mt_WBo_Bo.tail<3>();
    const auto Svv_WBo_Bo = Stt_WBo_Bo.topLeftCorner<3, 3>();
    const auto Sww_WBo_Bo = Stt_WBo_Bo.bottomRightCorner<3, 3>();
    const auto Svw_WBo_Bo = Stt_WBo_Bo.topRightCorner<3, 3>();

    // Linear velocity: re-expressing it in the base frame accounts for the
    // angular velocity and the lever arm. Angular velocity: only the rotation
    // applies. The cross-covariance follows from the same transformation.
    beluga_estimation::Vector3d Mv_WB_B;
    beluga_estimation::Vector3d Mw_WB_B;
    beluga_estimation::Matrix3d Svv_WB_B;
    beluga_estimation::Matrix3d Sww_WB_B;
    beluga_estimation::Matrix3d Svw_WB_B;
    if (X_BBo) {
      const auto R_BBo = X_BBo->rotationMatrix();
      const auto p_BBo = X_BBo->translation();
      // Materialize. The cross-product matrix is a temporary, and storing a
      // lazy product of it would dangle once the temporary is destroyed.
      const beluga_estimation::Matrix3d pR_BBo = ToCrossProductMatrix(p_BBo) * R_BBo;
      const beluga_estimation::Matrix3d R_Svw_WBo_Bo = R_BBo * Svw_WBo_Bo;
      Mv_WB_B = R_BBo * Mv_WBo_Bo + p_BBo.cross(R_BBo * Mw_WBo_Bo);
      Mw_WB_B = R_BBo * Mw_WBo_Bo;
      Svv_WB_B = R_BBo * Svv_WBo_Bo * R_BBo.transpose() + pR_BBo * Sww_WBo_Bo * pR_BBo.transpose() +
                 R_Svw_WBo_Bo * pR_BBo.transpose() + pR_BBo * R_Svw_WBo_Bo.transpose();
      Svw_WB_B = R_Svw_WBo_Bo * R_BBo.transpose() + pR_BBo * Sww_WBo_Bo * R_BBo.transpose();
      Sww_WB_B = R_BBo * Sww_WBo_Bo * R_BBo.transpose();
    } else {
      // No transform, the base frame B is the body frame Bo.
      Mv_WB_B = Mv_WBo_Bo;
      Mw_WB_B = Mw_WBo_Bo;
      Svv_WB_B = Svv_WBo_Bo;
      Sww_WB_B = Sww_WBo_Bo;
      Svw_WB_B = Svw_WBo_Bo;
    }

    // Create identifiers — variable name is shared across probes (sensor fusion),
    // constraint name is precomputed at construction from the configured names.
    Identifier variable_id{config_.velocity_variable_name, timestamp};

    auto candidate = context.dbn.template MaybeAdd<EuclideanVariable_<T>>(variable_id, NDim<3>{});
    if (!candidate) {
      return Err(CollectError(candidate));
    }
    auto variable = Unwrap(candidate);
    BELUGA_ESTIMATION_ASSERT(variable != nullptr);

    if (variable->Size() != 3) {
      auto error = Err(fmt::format(
          "variable '{}' has dimension {} but the odometry probe requires 3", variable_id.name, variable->Size()));
      middleware_->Error("OdomProbe: {}", error.value());
      return error;
    }
    middleware_->Debug(
        "OdomProbe: added velocity variable '{}' at timestamp {} from '{}' device", variable_id.name,
        variable_id.timestamp.time_since_epoch().count(), config_.device_name);

    // Assemble the planar twist [vx, vy, omega] and select the coordinates
    // enabled by the mask.
    beluga_estimation::Vector3d Mv_2d_WB_B;
    Mv_2d_WB_B << Mv_WB_B.head<2>(), Mw_WB_B(2);
    // The cross-covariance of planar linear velocity with the yaw rate is the
    // omega_z column of the linear-angular cross-covariance.
    const beluga_estimation::Vector2d Svw_2d_WB_B = Svw_WB_B.col(2).head<2>();
    beluga_estimation::Matrix3d Stt_2d_WB_B = beluga_estimation::Matrix3d::Zero();
    Stt_2d_WB_B.topLeftCorner<2, 2>() = Svv_WB_B.topLeftCorner<2, 2>();
    Stt_2d_WB_B.topRightCorner<2, 1>() = Svw_2d_WB_B;
    Stt_2d_WB_B.bottomLeftCorner<1, 2>() = Svw_2d_WB_B.transpose();
    Stt_2d_WB_B(2, 2) = Sww_WB_B(2, 2);

    // Project the twist distribution onto the fused components. For Gaussian
    // measurements this is an exact marginalization.
    auto distribution = std::make_shared<const NormalDistribution_<T>>(
        Project(Mv_2d_WB_B.template cast<T>(), config_.twist_mask),
        Project(Stt_2d_WB_B.template cast<T>(), config_.twist_mask));
    auto constraint = context.dbn.template Add<NormalLikelihoodConstraint_<T>>(
        Identifier{twist_constraint_name_, timestamp}, variable->Id(), std::move(distribution), twist_projection_);
    if (!constraint) {
      middleware_->Warn("Failed to add velocity constraint");
      return Err(CollectError(constraint));
    }
    middleware_->Debug(
        "OdomProbe: added velocity constraint '{}' at timestamp {} from '{}' device", twist_constraint_name_,
        timestamp.time_since_epoch().count(), config_.device_name);

    return Ok();
  }

  Configuration config_;
  std::shared_ptr<MiddlewareInterface> middleware_;
  MessagePipe<beluga_estimation::msg::Odometry> message_pipe_;
  bool use_pose_ = false;
  bool use_twist_ = false;
  beluga_estimation::MatrixX<T> twist_projection_;
  std::string pose_constraint_name_;
  std::string twist_constraint_name_;
};

using OdomProbe = OdomProbe_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation::OdomProbe, beluga_estimation::BayesianOperation);
