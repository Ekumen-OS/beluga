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
 * @brief Extended Kalman Filter (EKF) solver operation.
 *
 * Implements an EKF over a joint state of multiple tracked variables using
 * the factor graph defined by the DBN. The joint state lives on the product
 * manifold of the tracked variables' manifolds; its mean is stored in the
 * ambient space of that product manifold and its covariance in the tangent
 * space. This separation is essential for non-Euclidean variables, where
 * uncertainty and corrections are meaningful only locally.
 *
 * The filter is **stateful**: it keeps its own estimate internally and only
 * writes to the estimate store, never reading from it. State initialization
 * (bootstrap) is done from prior constraints on the earliest aligned chunk of
 * tracked variables; every tracked variable must have exactly one prior,
 * otherwise execution fails with `Error::kFailure`.
 *
 * Execution walks chronologically through aligned chunks of tracked
 * variables: for each chunk, process-model constraints (whose dependent
 * variables belong to the chunk) are applied first, then measurement
 * constraints (that condition on the chunk's variables). When no new aligned
 * chunk is available, execution is a no-op and reports success.
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <fmt/ranges.h>

#include <Eigen/Dense>
#include <range/v3/action/sort.hpp>
#include <range/v3/action/unique.hpp>
#include <range/v3/algorithm/all_of.hpp>
#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/bitset.hpp>
#include <beluga_estimation/common/format.hpp>
#include <beluga_estimation/common/interval.hpp>
#include <beluga_estimation/common/probability.hpp>
#include <beluga_estimation/common/ranges.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/core/locally_normal_constraint.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/distributions/generic_tangent_distribution.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/manifolds/product_manifold.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>

namespace beluga_estimation {

/**
 * @brief Extended Kalman Filter solver operation.
 *
 * The filter estimates a joint state @f$ \mathbf{x} \in \mathcal{M} @f$ living on a
 * product manifold @f$ \mathcal{M} = \mathcal{M}_1 \times \cdots \times \mathcal{M}_n @f$
 * and governed by the following nonlinear discrete-time dynamic system:
 *
 * @f[
 *   \mathbf{x}_{k} = f_k(\mathbf{x}_{k - 1}) \boxplus \mathbf{w}_k, \quad \mathbf{w}_k \sim \mathcal{N}(\mathbf{0},
 * \mathbf{Q}_k)
 * @f]
 * @f[
 *   \mathbf{z}^i_k = h^i_k(\mathbf{x}_{k}) + \mathbf{v}^i_k, \quad \mathbf{v}^i_k \sim \mathcal{N}(\mathbf{0},
 * \mathbf{R}^i_k)
 * @f]
 *
 * where @f$ f_k @f$ is the process model, @f$ h^i_k @f$ for @f$ i = 0, \dots, m-1 @f$ are the measurement models,
 * @f$ \mathbf{w}_k @f$ is process noise with covariance @f$ \mathbf{Q}_k @f$, and @f$ \mathbf{v}^i_k @f$ are
 * measurement noise with covariance @f$ \mathbf{R}^i_k @f$.
 *
 * The process model is assembled from constraints between disjoint sets of variables. Measurement models are applied
 * chronologically. The DBN structure must be consistent with the EKF formulation. Non-Markovian processes, unexplained
 * variables, and overconstrained relationships will be rejected and fail execution.
 *
 * The posterior over @f$ \mathbf{x}_k @f$ is represented as a concentrated Gaussian distribution in the manifold
 * tangent space:
 * @f[
 *   p(\mathbf{x}_k | \mathbf{z}_{1:k}) \sim \boldsymbol{\mu}_k \boxplus \mathcal{N}_{\tan}(\mathbf{0}, \mathbf{P}_k)
 * @f]
 * where @f$ \boldsymbol{\mu}_k @f$ is the mean in ambient space and @f$ \mathbf{P}_k @f$ is the covariance in tangent
 * space.
 *
 * Update equations follow standard EKF derivations in stabilized or Joseph form, on manifold.
 *
 * State propagates according to the linearized process:
 * @f[
 *   \boldsymbol{\mu}^-_{k|k-1} = f_k(\boldsymbol{\mu}_{k-1|k-1})
 * @f]
 * @f[
 *   \mathbf{P}^-_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}_k
 * @f]
 * where @f$ \boldsymbol{\mu}^-_{k|k-1} @f$ is the predicted mean, @f$ \mathbf{P}^-_{k|k-1} @f$ is the predicted
 * covariance, and @f$ \mathbf{F}_k = \nabla f_k(\boldsymbol{\mu}_{k-1|k-1}) @f$ is the Jacobian of the process model.
 *
 * Then it is corrected by each measurement @f$ \mathbf{z}^i_k @f$. Given:
 * @f[
 *   \mathbf{S}^i_k = \mathbf{H}^i_k ^{i-1}\mathbf{P}^-_{k|k-1} \mathbf{H}^{i T}_k + \mathbf{R}^i_k
 * @f]
 * @f[
 *   \mathbf{K}^i_k = ^{i-1}\mathbf{P}^-_{k|k-1} \mathbf{H}^{i T}_k (\mathbf{S}^i_k)^{-1}
 * @f]
 * where @f$ \mathbf{K}^i_k @f$ is the Kalman gain and @f$ \mathbf{S}^i_k @f$ is the innovation covariance,
 * and @f$ \mathbf{H}^i_k = \nabla h^i_k(\boldsymbol{\mu}_{k|k-1}) @f$ is the Jacobian of the corresponding
 * measurement model, state is updated in sequence:
 * @f[
 *   ^{i}\boldsymbol{\mu}^+_{k|k} = ^{i-1}\boldsymbol{\mu}^-_{k|k-1} \boxplus \mathbf{K}^i_k (\mathbf{z}^i_k - h^i_k(
 * ^{i-1}\boldsymbol{\mu}^-_{k|k-1}))
 * @f]
 * @f[
 *   ^{i}\mathbf{P}^+_{k|k} = ^{i-1}\mathbf{P}^-_{k|k-1} - \mathbf{K}^i_k \mathbf{S}^i_k \mathbf{K}^{i T}_k
 * @f]
 * where @f$ ^{i}\boldsymbol{\mu}^+_{k|k} @f$ is the posterior mean and @f$ ^{i}\mathbf{P}^+_{k|k} @f$ is the posterior
 * covariance\ after the i-th measurement update.
 *
 * For further reference, see section 13.2.3, Simon, D. (2006). Optimal state estimation: Kalman, H infinity, and
 * nonlinear approaches. John Wiley & Sons.
 *
 * ## Implementation Notes
 *
 * The filter is fully configurable: it tracks a set of named variables (see @ref
 * Configuration::tracked_variable_names). It is also stateful: it maintains the joint estimate across calls and writes
 * it to the estimate store under
 * @ref Configuration::estimate_name, without ever reading the store back.
 *
 * @tparam T Scalar type (e.g. `float` or `double`).
 */
template <typename T>
class ExtendedKalmanFilter_ : public BayesianOperation_<T> {
 public:
  struct Configuration {
    /**
     * @brief Declares and loads the solver parameters.
     *
     * The supplied prefix is concatenated directly with parameter names; callers
     * should include a trailing separator when their parameter hierarchy requires
     * one. An empty tracked-variable list is invalid and causes execution to
     * return an error with `Error::kFailure` during initialization.
     *
     * @param middleware Parameter provider used to declare and retrieve values.
     * @param prefix Namespace prefix for this operation's parameters.
     * @return Configuration populated with declared parameters and defaults.
     */
    static Configuration LoadFromParams(
        std::shared_ptr<MiddlewareInterface> middleware,
        const std::string& prefix = "") {
      Configuration config;
      config.tracked_variable_names =
          middleware->DeclareParameter<std::vector<std::string>>(prefix + "tracked.variables", {});
      config.estimate_name = middleware->DeclareParameter<std::string>(prefix + "estimate.name", "main");
      return config;
    }

    /// Names of variables forming the ordered joint state.
    std::vector<std::string> tracked_variable_names;

    /// Name under which each published joint estimate is stored.
    std::string estimate_name = "main";
  };

  /**
   * @brief Creates the solver from middleware parameters.
   *
   * @param middleware Parameter provider.
   * @param prefix Namespace prefix for this operation's parameters.
   * @return A result containing the solver instance.
   */
  static Result_<std::unique_ptr<ExtendedKalmanFilter_<T>>> Make(
      std::shared_ptr<MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<ExtendedKalmanFilter_<T>>(std::move(middleware), prefix);
  }

  /**
   * @brief Constructs the solver from middleware parameters.
   *
   * The middleware is used at construction time to declare parameters and
   * afterwards to report diagnostics; filtering itself operates exclusively on
   * the supplied execution context.
   *
   * @param middleware Parameter provider and logging sink.
   * @param prefix Namespace prefix for this operation's parameters.
   */
  ExtendedKalmanFilter_(std::shared_ptr<MiddlewareInterface> middleware, std::string_view prefix)
      : ExtendedKalmanFilter_(middleware, Configuration::LoadFromParams(middleware, std::string(prefix))) {}

  /**
   * @brief Constructs the solver with an explicit configuration.
   *
   * This overload is useful for programmatic composition and tests that do not
   * need middleware-backed parameter declaration.
   *
   * @param middleware Middleware handle used to report diagnostics.
   * @param config Names and output estimate name for this solver.
   */
  ExtendedKalmanFilter_(std::shared_ptr<MiddlewareInterface> middleware, Configuration config)
      : middleware_(std::move(middleware)), config_(std::move(config)) {}

  /**
   * @brief Advances the estimate through every complete, unseen time slice.
   *
   * Initialization uses priors at the earliest timestamp. Later invocations
   * begin at the current estimate timestamp, so repeated calls are harmless when
   * the DBN has not received another aligned set of tracked variables. Each time
   * slice is transactional: an unsuccessful prediction or update restores the
   * preceding estimate before execution stops.
   *
   * @param context Provides the DBN input and estimate store output.
   * @param budget Unused.
   * @return Success, or an error with `Error::kFailure` when the graph
   *   cannot represent the required EKF structure.
   */
  Result Execute(Context_<T>& context, const ComputeBudget& /*budget*/) override {
    bool got_new_estimate = false;

    if (!state_) {
      state_ = BuildState(context, config_.tracked_variable_names);
      if (!state_) {
        middleware_->Error(
            "ExtendedKalmanFilter: could not bootstrap the EKF state from tracked variable(s) [{}]",
            fmt::join(config_.tracked_variable_names, ", "));
        return Err(Error::kFailure, "failed to bootstrap the EKF state");
      }
      got_new_estimate = true;
      middleware_->Debug(
          "ExtendedKalmanFilter: bootstrapped from {} tracked variable(s)", config_.tracked_variable_names.size());
      middleware_->Debug(
          "ExtendedKalmanFilter: joint state has ambient size {} and tangent size {} with seed variables [{}]",
          state_->structure.manifold->AmbientSize(), state_->structure.manifold->TangentSize(),
          fmt::join(state_->structure.seed_variable_ids, ", "));
      middleware_->Debug(
          "ExtendedKalmanFilter: bootstrap estimate at timestamp {} has mean {} and covariance diag {}",
          state_->estimate.timestamp.time_since_epoch().count(), VectorLike(state_->estimate.mean),
          MatrixLike(state_->estimate.covariance, MatrixStyle::kDiagonal));
    }

    // Transposition forms chronological slices only when every tracked variable
    // is present, preventing a partial joint state from being filtered.
    const auto interval = TimeInterval::Since(state_->estimate.timestamp);
    auto timeline = config_.tracked_variable_names | ranges::views::transform([&](const auto& name) {
                      return context.dbn.GetVariables(name, interval) |
                             ranges::view::transform([](const auto& variable) { return variable->Id(); });
                    }) |
                    beluga_estimation::views::materialized_transpose;

    Result status;
    std::size_t num_chunks = 0;
    for (const auto& chunk : timeline) {
      // Preserve the last consistent posterior as the rollback point for this slice.
      state_->prev_estimate = state_->estimate;

      auto common_timestamp = GetCommonTimestamp(chunk);
      if (!common_timestamp) {
        middleware_->Error(
            "ExtendedKalmanFilter: tracked variables do not share a common timestamp: [{}]", fmt::join(chunk, ", "));
        return Err(Error::kFailure, "tracked variables do not share a common timestamp");
      }
      state_->estimate.timestamp = common_timestamp.value();

      middleware_->Debug(
          "ExtendedKalmanFilter: processing state chunk {} at timestamp {}", num_chunks,
          state_->estimate.timestamp.time_since_epoch().count());

      // Propagate first; measurements at this timestamp condition that prediction.
      status = ProcessPredictions(context, chunk, *state_);
      if (!status) {
        middleware_->Error(
            "ExtendedKalmanFilter: prediction failed at timestamp {}: {}; rolling back to timestamp {}",
            state_->estimate.timestamp.time_since_epoch().count(), status.error(),
            state_->prev_estimate.timestamp.time_since_epoch().count());
        state_->estimate = state_->prev_estimate;
        break;
      }

      status = ProcessUpdates(context, chunk, *state_);
      if (!status) {
        middleware_->Error(
            "ExtendedKalmanFilter: measurement update failed at timestamp {}: {}; rolling back to timestamp {}",
            state_->estimate.timestamp.time_since_epoch().count(), status.error(),
            state_->prev_estimate.timestamp.time_since_epoch().count());
        state_->estimate = state_->prev_estimate;
        break;
      }

      got_new_estimate = true;
      num_chunks++;
      middleware_->Debug(
          "ExtendedKalmanFilter: posterior estimate at timestamp {} has mean {} and covariance diag {}",
          state_->estimate.timestamp.time_since_epoch().count(), VectorLike(state_->estimate.mean),
          MatrixLike(state_->estimate.covariance, MatrixStyle::kDiagonal));
    }

    if (num_chunks == 0 && status) {
      middleware_->Debug(
          "ExtendedKalmanFilter: no new aligned state chunk after timestamp {} for tracked variable(s) [{}]",
          state_->estimate.timestamp.time_since_epoch().count(), fmt::join(config_.tracked_variable_names, ", "));
    }

    if (got_new_estimate) {
      // Publish a fresh estimate object rather than using the store as filter state.
      auto embedded_distribution = std::make_shared<NormalDistribution_<T>>(state_->estimate.covariance);
      auto tangent_distribution = std::make_shared<GenericTangentDistribution_<T>>(
          state_->structure.manifold, std::move(embedded_distribution), state_->estimate.mean);
      auto estimate_variables = config_.tracked_variable_names | ranges::views::transform([&](const std::string& name) {
                                  return context.dbn.GetVariable(Identifier{name, state_->estimate.timestamp});
                                }) |
                                ranges::to_vector;
      auto estimate =
          std::make_shared<OnManifoldEstimate_<T>>(std::move(estimate_variables), std::move(tangent_distribution));
      auto estimate_id = Identifier{config_.estimate_name, state_->estimate.timestamp};
      context.estimates.Add(std::move(estimate_id), std::move(estimate), MatchScope::kSemantic);
      middleware_->Debug(
          "ExtendedKalmanFilter: published estimate at timestamp {} after {} state chunk(s)",
          state_->estimate.timestamp.time_since_epoch().count(), num_chunks);
      middleware_->Debug(
          "ExtendedKalmanFilter: published estimate at timestamp {} has mean {} and covariance diag {}",
          state_->estimate.timestamp.time_since_epoch().count(), VectorLike(state_->estimate.mean),
          MatrixLike(state_->estimate.covariance, MatrixStyle::kDiagonal));
    }

    return status;
  }

  /**
   * @brief Discards the internal posterior and reusable work buffers.
   *
   * The next call to @ref Execute bootstraps again from the DBN's earliest
   * tracked variables.
   *
   * @param budget Unused.
   * @return Success.
   */
  Result Finalize(const ComputeBudget& /*budget*/) override {
    state_.reset();
    return Ok();
  }

 private:
  /// Immutable layout and initialization metadata for the joint state.
  struct StateStructure {
    /// Maps each configured variable name to its component of the product manifold.
    std::unordered_map<std::string, std::size_t> variable_arrangement;

    /// Product manifold defining the joint state's ambient and tangent coordinates.
    std::shared_ptr<const ProductManifold_<T>> manifold;

    /// Earliest instances used to build the filter structure.
    /// Also used to locate the priors that bootstrap filter state.
    std::vector<Identifier> seed_variable_ids;
  };

  /// Gaussian posterior at one timestamp, expressed in the joint manifold's charts.
  struct StateEstimate {
    /// Timestamp shared by all variables represented by this posterior.
    TimePoint timestamp;

    /// Joint mean in ambient coordinates, as required by manifold operations.
    beluga_estimation::VectorX<T> mean;

    /// Joint uncertainty in tangent coordinates around @ref mean.
    beluga_estimation::MatrixX<T> covariance;
  };

  /**
   * @brief Reusable buffers for filtering and bootstrapping.
   *
   * The workspace keeps dynamic allocations out of the steady-state filtering
   * path. Measurement buffers grow to accommodate the largest residual observed
   * so far, while process and prior buffers have the fixed size of the joint
   * state.
   */
  struct StateWorkspace {
    /// Product-manifold components selected by the current constraint.
    std::vector<std::size_t> conditioning_variable_indices;

    /// Product-manifold components produced by the current process constraint.
    std::vector<std::size_t> dependent_variable_indices;

    /// Coverage tracker used to ensure process constraints partition the target state.
    std::vector<bool> dependent_variables_seen;

    /// Scratch pad for a prior mean before it is lifted into the joint state.
    beluga_estimation::VectorX<T> prior_mean;

    /// Scratch pad for a prior covariance before it is lifted into the joint state.
    beluga_estimation::MatrixX<T> prior_covariance;

    /// Local process-model mean before it is placed in the joint state.
    beluga_estimation::VectorX<T> subprocess_mean;

    /// Local process noise in the dependent variables' tangent coordinates.
    beluga_estimation::MatrixX<T> subprocess_covariance;

    /// Local transition sensitivity from conditioning to dependent coordinates.
    beluga_estimation::MatrixX<T> subprocess_jacobian;

    /// Assembled predicted mean in the complete product manifold.
    beluga_estimation::VectorX<T> process_mean;

    /// Assembled process noise in complete tangent coordinates.
    beluga_estimation::MatrixX<T> process_covariance;

    /// Assembled transition Jacobian for covariance propagation.
    beluga_estimation::MatrixX<T> process_jacobian;

    /// Innovation for the measurement currently being assimilated.
    beluga_estimation::VectorX<T> measurement_residual;

    /// Covariance of the measurement currently being assimilated.
    beluga_estimation::MatrixX<T> measurement_covariance;

    /// Measurement sensitivity in its selected local tangent coordinates.
    beluga_estimation::MatrixX<T> measurement_jacobian;

    /// Local measurement sensitivity embedded in the complete tangent space.
    beluga_estimation::MatrixX<T> full_measurement_jacobian;

    /// Identity over the complete tangent space, reused by the Joseph update.
    beluga_estimation::MatrixX<T> identity;

    /// Reserved workspace for innovation covariance calculations.
    beluga_estimation::MatrixX<T> innovation_covariance;

    /// Reserved workspace for the Joseph covariance-update multiplier.
    beluga_estimation::MatrixX<T> update_multiplier;

    /// Reserved workspace for the Kalman gain.
    beluga_estimation::MatrixX<T> kalman_gain;

    /// Tangent-space correction applied to the manifold-valued mean.
    beluga_estimation::VectorX<T> correction_term;
  };

  /// Persistent state required to continue filtering across Execute calls.
  struct State {
    StateStructure structure;

    /// Reusable buffers that keep dynamic allocations out of the filtering path.
    StateWorkspace workspace;

    /// Most recently accepted posterior.
    StateEstimate estimate;

    /// Prior estimate by the end of each time slice.
    StateEstimate prev_estimate;
  };

  /**
   * @brief Translates constraint variable names into product-manifold component indices.
   *
   * Variable instances may have different timestamps, but all must name a
   * configured state component. The returned ordering matches @p variable_ids
   * and is therefore suitable for the corresponding constraint's values and
   * Jacobian axes.
   *
   * @param structure Joint-state layout.
   * @param variable_ids Variables referenced by a constraint.
   * @param indices Destination for the matching component indices.
   * @return `true` if every variable belongs to the tracked joint state.
   */
  static bool GetVariableIndices(
      const StateStructure& structure,
      std_future::span<const Identifier> variable_ids,
      std::vector<std::size_t>& indices) {
    indices.clear();
    for (const auto& id : variable_ids) {
      auto it = structure.variable_arrangement.find(id.name);
      if (it == structure.variable_arrangement.end()) {
        return false;
      }
      indices.push_back(it->second);
    }
    return true;
  }

  /**
   * @brief Verifies that a collection of variables represents one time slice.
   *
   * Aligned timestamps are the temporal invariant that permits the variables to
   * be treated as a single joint state.
   *
   * @param variable_ids Candidate time slice.
   * @return Its shared timestamp, or `std::nullopt` for an empty or misaligned slice.
   */
  static std::optional<TimePoint> GetCommonTimestamp(const std::vector<Identifier>& variable_ids) {
    if (variable_ids.empty()) {
      return std::nullopt;
    }
    auto timestamp = variable_ids.front().timestamp;
    for (const auto& id : variable_ids) {
      if (id.timestamp != timestamp) {
        return std::nullopt;
      }
    }
    return timestamp;
  }

  /**
   * @brief Builds the fixed product-manifold layout from earliest variables in the network.
   *
   * The configured order is preserved in the product manifold and consequently
   * defines the component order of all joint means, covariances, and estimates.
   *
   * @param structure Destination state layout.
   * @param context Bears the DBN containing the tracked variables.
   * @param variable_names Ordered names configured for the filter.
   * @return `true` if a valid filter structure was successfully built, `false` otherwise.
   */
  bool
  BuildStructureInto(StateStructure& structure, Context_<T>& context, const std::vector<std::string>& variable_names) {
    if (variable_names.empty()) {
      middleware_->Debug("ExtendedKalmanFilter: no tracked variables were configured");
      return false;
    }
    ProductManifoldBuilder_<T> builder;
    structure.variable_arrangement.reserve(variable_names.size());
    structure.seed_variable_ids.reserve(variable_names.size());
    for (const auto& [index, name] : variable_names | ranges::views::enumerate) {
      const auto variable = context.dbn.GetEarliestVariable(name);
      if (!variable) {
        middleware_->Debug("ExtendedKalmanFilter: no variable named '{}' in the network", name);
        return false;
      }
      structure.variable_arrangement.emplace(name, index);
      structure.seed_variable_ids.push_back(variable->Id());
      builder.Add(variable->Manifold());
      middleware_->Debug(
          "ExtendedKalmanFilter: joint state component {} is '{}' seeded from '{}'", index, name, variable->Id());
    }
    structure.manifold = builder.Build();
    return true;
  }

  /**
   * @brief Retrieves the unique locally normal prior that initializes a variable.
   *
   * EKF bootstrapping requires a non-conditional distribution. Multiple incoming
   * constraints are rejected rather than arbitrarily selecting one.
   *
   * @param context DBN to query.
   * @param variable_id Initial variable instance.
   * @return The unique compatible prior, or `nullptr` when the requirement is unmet.
   */
  std::shared_ptr<const LocallyNormalConstraint_<T>> GetPriorConstraint(
      const Context_<T>& context,
      const Identifier& variable_id) {
    auto constraints = context.dbn.GetConditioningConstraints(variable_id);
    auto it = ranges::begin(constraints);
    if (it == ranges::end(constraints)) {
      middleware_->Debug(
          "ExtendedKalmanFilter: variable '{}' has no conditioning constraints to use as a prior", variable_id);
      return nullptr;
    }
    auto constraint = std::dynamic_pointer_cast<const LocallyNormalConstraint_<T>>(*it);
    if (!constraint) {
      middleware_->Debug(
          "ExtendedKalmanFilter: prior candidate '{}' for variable '{}' is not locally normal", (*it)->Id(),
          variable_id);
      return nullptr;
    }
    auto is_prior = !constraint->HasConditioningVariables() && constraint->HasDependentVariables();
    if (!is_prior) {
      middleware_->Debug(
          "ExtendedKalmanFilter: constraint '{}' is not a prior for variable '{}'", constraint->Id(), variable_id);
      return nullptr;
    }
    if (++it != ranges::end(constraints)) {
      middleware_->Debug(
          "ExtendedKalmanFilter: variable '{}' has more than one prior; cannot bootstrap unambiguously", variable_id);
      return nullptr;
    }
    return constraint;
  }

  /**
   * @brief Initializes a joint Gaussian posterior from the seed variables' priors.
   *
   * Individual prior moments are lifted into the joint product manifold. The
   * operation requires non-overlapping dependent components that collectively
   * cover the state.
   *
   * @param estimate Destination posterior.
   * @param context Bearing the DBN containing initialization priors.
   * @param structure Fixed joint-state layout.
   * @param workspace Scratch buffers for the per-prior moments.
   * @return `true` if a complete, aligned initialization can be assembled.
   */
  bool BuildEstimateInto(
      StateEstimate& estimate,
      Context_<T>& context,
      const StateStructure& structure,
      StateWorkspace& workspace) {
    auto common_timestamp = GetCommonTimestamp(structure.seed_variable_ids);
    if (!common_timestamp) {
      middleware_->Debug(
          "ExtendedKalmanFilter: seed variables are not aligned to a common timestamp: [{}]",
          fmt::join(structure.seed_variable_ids, ", "));
      return false;
    }

    estimate.timestamp = common_timestamp.value();
    estimate.mean.resize(static_cast<Eigen::Index>(structure.manifold->AmbientSize()));
    estimate.covariance.resize(
        static_cast<Eigen::Index>(structure.manifold->TangentSize()),
        static_cast<Eigen::Index>(structure.manifold->TangentSize()));
    // Prior contributions are lifted component-wise below; lifting leaves
    // non-selected dimensions untouched, so the joint moment buffers must start
    // zeroed. Otherwise uninitialized entries leak into the posterior.
    estimate.mean.setZero();
    estimate.covariance.setZero();

    // Each prior may cover several components, but together they must cover the joint state exactly once.
    ClearBitSet(workspace.dependent_variables_seen);
    for (const auto& variable_id : structure.seed_variable_ids) {
      auto prior_constraint = GetPriorConstraint(context, variable_id);
      if (!prior_constraint) {
        middleware_->Debug("ExtendedKalmanFilter: no usable prior for seed variable '{}'", variable_id);
        return false;
      }

      const bool valid_dependent_variable_indices =
          GetVariableIndices(structure, prior_constraint->DependentVariables(), workspace.dependent_variable_indices);
      if (!valid_dependent_variable_indices) {
        // Some of the prior's dependent variables are not part of the configured joint state.
        middleware_->Debug(
            "ExtendedKalmanFilter: prior '{}' predicts untracked variable(s) [{}]", prior_constraint->Id(),
            fmt::join(prior_constraint->DependentVariables(), ", "));
        return false;
      }

      const bool duplicate_dependent_variables =
          UpdateBitSet(workspace.dependent_variable_indices, workspace.dependent_variables_seen);
      if (duplicate_dependent_variables) {
        // More than one prior covers the same component of the joint state.
        // Cannot decide which to use, so the filter cannot be initialized.
        middleware_->Debug(
            "ExtendedKalmanFilter: prior '{}' overlaps components already covered by another prior",
            prior_constraint->Id());
        return false;
      }

      const auto prior_state_ambient_size =
          static_cast<Eigen::Index>(structure.manifold->AmbientSize(workspace.dependent_variable_indices));
      beluga_estimation::Ref<beluga_estimation::VectorX<T>> prior_mean_block =
          workspace.prior_mean.head(prior_state_ambient_size);
      prior_constraint->CalcMean({}, to_view(prior_mean_block));
      structure.manifold->LiftAmbient(
          to_view(prior_mean_block), workspace.dependent_variable_indices, to_view(estimate.mean));

      const auto prior_state_tangent_size =
          static_cast<Eigen::Index>(structure.manifold->TangentSize(workspace.dependent_variable_indices));
      beluga_estimation::Ref<beluga_estimation::MatrixX<T>> prior_covariance_block =
          workspace.prior_covariance.topLeftCorner(prior_state_tangent_size, prior_state_tangent_size);
      prior_constraint->CalcCovariance({}, to_view(prior_covariance_block));

      structure.manifold->LiftTangent(
          to_view(prior_covariance_block), workspace.dependent_variable_indices, to_view(estimate.covariance));

      middleware_->Debug(
          "ExtendedKalmanFilter: prior '{}' has mean {} and covariance diag {}", prior_constraint->Id(),
          VectorLike(prior_mean_block), MatrixLike(prior_covariance_block, MatrixStyle::kDiagonal));
    }
    BELUGA_ESTIMATION_ASSERT(IsCovarianceMatrix(estimate.covariance));
    if (!AllBitSet(workspace.dependent_variables_seen)) {
      middleware_->Debug("ExtendedKalmanFilter: priors do not cover every component of the joint state");
      return false;
    }
    return true;
  }

  /**
   * @brief Constructs the persistent state used by the filter.
   * @param context Bearing the DBN that provides the initial variables and priors.
   * @param variable_names Ordered names forming the joint state.
   * @return Initialized state, or `nullptr` if bootstrap validation fails.
   */
  std::unique_ptr<State> BuildState(Context_<T>& context, const std::vector<std::string>& variable_names) {
    auto state = std::make_unique<State>();

    if (!BuildStructureInto(state->structure, context, variable_names)) {
      middleware_->Debug("ExtendedKalmanFilter: failed to build the joint state structure");
      return nullptr;
    }

    if (!BuildWorkspaceInto(state->workspace, state->structure)) {
      middleware_->Debug("ExtendedKalmanFilter: failed to size the filtering workspace");
      return nullptr;
    }

    if (!BuildEstimateInto(state->estimate, context, state->structure, state->workspace)) {
      middleware_->Debug("ExtendedKalmanFilter: failed to assemble the initial estimate from priors");
      return nullptr;
    }

    return state;
  }

  /**
   * @brief Sizes the workspace buffers for the fixed joint-state layout.
   *
   * The identity matrix is retained for covariance update.
   *
   * @param workspace Workspace to size.
   * @param structure Joint-state layout that determines fixed buffer dimensions.
   * @return `true` on success.
   */
  static bool BuildWorkspaceInto(StateWorkspace& workspace, const StateStructure& structure) {
    workspace.conditioning_variable_indices.reserve(structure.variable_arrangement.size());
    workspace.dependent_variable_indices.reserve(structure.variable_arrangement.size());
    workspace.dependent_variables_seen.resize(structure.variable_arrangement.size());
    workspace.prior_mean.resize(static_cast<Eigen::Index>(structure.manifold->AmbientSize()));
    workspace.prior_covariance.resize(
        static_cast<Eigen::Index>(structure.manifold->TangentSize()),
        static_cast<Eigen::Index>(structure.manifold->TangentSize()));
    workspace.subprocess_mean.resize(static_cast<Eigen::Index>(structure.manifold->AmbientSize()));
    workspace.subprocess_covariance.resize(
        static_cast<Eigen::Index>(structure.manifold->TangentSize()),
        static_cast<Eigen::Index>(structure.manifold->TangentSize()));
    workspace.subprocess_jacobian.resize(
        static_cast<Eigen::Index>(structure.manifold->TangentSize()),
        static_cast<Eigen::Index>(structure.manifold->TangentSize()));
    workspace.process_mean.resize(static_cast<Eigen::Index>(structure.manifold->AmbientSize()));
    workspace.process_covariance.resize(
        static_cast<Eigen::Index>(structure.manifold->TangentSize()),
        static_cast<Eigen::Index>(structure.manifold->TangentSize()));
    workspace.process_jacobian.resize(
        static_cast<Eigen::Index>(structure.manifold->TangentSize()),
        static_cast<Eigen::Index>(structure.manifold->TangentSize()));
    workspace.identity = beluga_estimation::MatrixX<T>::Identity(
        static_cast<Eigen::Index>(structure.manifold->TangentSize()),
        static_cast<Eigen::Index>(structure.manifold->TangentSize()));
    workspace.correction_term.resize(static_cast<Eigen::Index>(structure.manifold->TangentSize()));
    return true;
  }

  /**
   * @brief Applies the Markov process models for a time slice.
   *
   * Each process constraint must be locally normal, depend only on the previous
   * posterior timestamp, and produce a disjoint subset of the current joint
   * state. The combined transition Jacobian propagates covariance in tangent
   * coordinates; process noise is then added in those same coordinates.
   *
   * @param context Bearing the DBN containing process constraints.
   * @param variable_ids Aligned variables at the target timestamp.
   * @param state Persistent posterior, joint-state layout, and workspace.
   * @return Success, otherwise an error with `Error::kFailure`.
   */
  Result ProcessPredictions(const Context_<T>& context, const std::vector<Identifier>& variable_ids, State& state) {
    state.workspace.process_mean.setZero();
    state.workspace.process_covariance.setZero();
    state.workspace.process_jacobian.setZero();
    ClearBitSet(state.workspace.dependent_variables_seen);

    for (const auto& constraint : context.dbn.GetConditioningConstraints(variable_ids)) {
      // Priors belong only to bootstrap and are not process models.
      if (!constraint->HasConditioningVariables()) {
        continue;
      }

      auto normal_process_constraint = std::dynamic_pointer_cast<const LocallyNormalConstraint_<T>>(constraint);
      if (!normal_process_constraint) {
        middleware_->Error("ExtendedKalmanFilter: process constraint '{}' is not locally normal", constraint->Id());
        return Err(Error::kFailure, "process constraint is not locally normal");
      }

      middleware_->Debug(
          "ExtendedKalmanFilter: applying process constraint '{}' conditioning on [{}] to predict [{}]",
          constraint->Id(), fmt::join(constraint->ConditioningVariables(), ", "),
          fmt::join(constraint->DependentVariables(), ", "));

      // The EKF state contains only one predecessor, so longer temporal links
      // cannot be evaluated without augmenting the state.
      const bool non_markov_process = ranges::any_of(
          normal_process_constraint->ConditioningVariables(),
          [&](const Identifier& id) { return id.timestamp != state.prev_estimate.timestamp; });
      if (non_markov_process) {
        middleware_->Error(
            "ExtendedKalmanFilter: process constraint '{}' conditions on [{}] but the previous posterior is at "
            "timestamp {}",
            constraint->Id(), fmt::join(constraint->ConditioningVariables(), ", "),
            state.prev_estimate.timestamp.time_since_epoch().count());
        return Err(Error::kFailure, "process constraint is not Markovian");
      }

      const bool valid_conditioning_variable_indices = GetVariableIndices(
          state.structure, normal_process_constraint->ConditioningVariables(),
          state.workspace.conditioning_variable_indices);
      if (!valid_conditioning_variable_indices) {
        return Err(Error::kFailure, "process constraint references untracked conditioning variables");
      }

      const bool valid_dependent_variable_indices = GetVariableIndices(
          state.structure, normal_process_constraint->DependentVariables(), state.workspace.dependent_variable_indices);
      if (!valid_dependent_variable_indices) {
        return Err(Error::kFailure, "process constraint references untracked dependent variables");
      }

      // Process constraints must partition the target joint state.
      const bool process_overlap =
          UpdateBitSet(state.workspace.dependent_variable_indices, state.workspace.dependent_variables_seen);
      if (process_overlap) {
        return Err(Error::kFailure, "process constraints overlap on the joint state");
      }

      const auto conditioning_state_values = state.structure.manifold->EmbedAmbient(
          to_view(state.prev_estimate.mean), state.workspace.conditioning_variable_indices);

      // Each constraint defines a subprocess affecting a unique subset of dependent variables.
      // All subprocesses' mean, covariance, and jacobian w.r.t. the conditioning variables need
      // to be lifted to joint state and assembled before the single prediction step can be carried out.
      const auto dependent_state_ambient_size =
          static_cast<Eigen::Index>(state.structure.manifold->AmbientSize(state.workspace.dependent_variable_indices));
      beluga_estimation::Ref<beluga_estimation::VectorX<T>> subprocess_mean_block =
          state.workspace.subprocess_mean.head(dependent_state_ambient_size);

      const auto dependent_state_tangent_size =
          static_cast<Eigen::Index>(state.structure.manifold->TangentSize(state.workspace.dependent_variable_indices));
      beluga_estimation::Ref<beluga_estimation::MatrixX<T>> subprocess_covariance_block =
          state.workspace.subprocess_covariance.topLeftCorner(
              dependent_state_tangent_size, dependent_state_tangent_size);

      const auto conditioning_state_tangent_size = static_cast<Eigen::Index>(
          state.structure.manifold->TangentSize(state.workspace.conditioning_variable_indices));
      beluga_estimation::Ref<beluga_estimation::MatrixX<T>> subprocess_jacobian_block =
          state.workspace.subprocess_jacobian.topLeftCorner(
              dependent_state_tangent_size, conditioning_state_tangent_size);

      normal_process_constraint->CalcMean(
          conditioning_state_values, to_view(subprocess_mean_block), to_view(subprocess_jacobian_block));
      normal_process_constraint->CalcCovariance(conditioning_state_values, to_view(subprocess_covariance_block));

      state.structure.manifold->LiftAmbient(
          to_view(subprocess_mean_block), state.workspace.dependent_variable_indices,
          to_view(state.workspace.process_mean));

      state.structure.manifold->LiftTangent(
          to_view(subprocess_covariance_block), state.workspace.dependent_variable_indices,
          to_view(state.workspace.process_covariance));

      state.structure.manifold->LiftTangent(
          to_view(subprocess_jacobian_block), state.workspace.dependent_variable_indices,
          state.workspace.conditioning_variable_indices, to_view(state.workspace.process_jacobian));
    }
    BELUGA_ESTIMATION_ASSERT(IsCovarianceMatrix(state.workspace.process_covariance));

    if (!AllBitSet(state.workspace.dependent_variables_seen)) {
      middleware_->Error(
          "ExtendedKalmanFilter: process constraints only cover joint state component(s) [{}] out of {}",
          fmt::join(IndexBitSet(state.workspace.dependent_variables_seen), ", "),
          state.workspace.dependent_variables_seen.size());
      return Err(Error::kFailure, "process constraints do not cover the joint state");
    }

    state.estimate.mean = state.workspace.process_mean;
    state.estimate.covariance = state.workspace.process_jacobian * state.prev_estimate.covariance *
                                    state.workspace.process_jacobian.transpose() +
                                state.workspace.process_covariance;

    middleware_->Debug(
        "ExtendedKalmanFilter: predicted mean {} with covariance diag {}", VectorLike(state.estimate.mean),
        MatrixLike(state.estimate.covariance, MatrixStyle::kDiagonal));
    return Ok();
  }

  /**
   * @brief Conditions the predicted posterior on measurements at a time slice.
   *
   * Measurement constraints have conditioning variables but no dependent
   * variables. Their local Jacobians are lifted into the joint tangent space
   * before the gain is calculated. Covariance uses the Joseph form to improve
   * numerical stability, and mean corrections use manifold addition rather than
   * ambient vector addition.
   *
   * @param context Bearing the DBN containing measurement constraints.
   * @param variable_ids Aligned variables at the current timestamp.
   * @param state Predicted posterior to update.
   * @return Success, otherwise an error with `Error::kFailure`.
   */
  Result ProcessUpdates(const Context_<T>& context, const std::vector<Identifier>& variable_ids, State& state) {
    for (const auto& constraint : context.dbn.GetConditionedConstraints(variable_ids)) {
      // Constraints producing another variable are processes, not observations.
      if (constraint->HasDependentVariables()) {
        continue;
      }

      auto normal_measurement_constraint = std::dynamic_pointer_cast<const LocallyNormalConstraint_<T>>(constraint);
      if (!normal_measurement_constraint) {
        middleware_->Error("ExtendedKalmanFilter: measurement constraint '{}' is not locally normal", constraint->Id());
        return Err(Error::kFailure, "measurement constraint is not locally normal");
      }

      middleware_->Debug(
          "ExtendedKalmanFilter: applying measurement constraint '{}' conditioning on [{}]", constraint->Id(),
          fmt::join(normal_measurement_constraint->ConditioningVariables(), ", "));

      const bool valid_conditioning_variable_indices = GetVariableIndices(
          state.structure, normal_measurement_constraint->ConditioningVariables(),
          state.workspace.conditioning_variable_indices);
      if (!valid_conditioning_variable_indices) {
        middleware_->Error(
            "ExtendedKalmanFilter: measurement constraint '{}' conditions on untracked variable(s) [{}]",
            constraint->Id(), fmt::join(normal_measurement_constraint->ConditioningVariables(), ", "));
        return Err(Error::kFailure, "measurement constraint references untracked conditioning variables");
      }

      const auto conditioning_state_values = state.structure.manifold->EmbedAmbient(
          to_view(state.estimate.mean), state.workspace.conditioning_variable_indices);

      // Each measurement introduces a correction. Each residual (or innovations) and its jacobian w.r.t. the
      // conditioning variables needs to be lifted to the joint state before the measurement update can be carried out.
      const auto residual_size = static_cast<Eigen::Index>(normal_measurement_constraint->ResidualSize());
      if (state.workspace.measurement_residual.size() < residual_size) {
        state.workspace.measurement_residual.resize(residual_size);
      }
      beluga_estimation::Ref<beluga_estimation::VectorX<T>> measurement_residual_block =
          state.workspace.measurement_residual.head(residual_size);

      if (state.workspace.measurement_covariance.rows() < residual_size ||
          state.workspace.measurement_covariance.cols() < residual_size) {
        state.workspace.measurement_covariance.resize(residual_size, residual_size);
      }
      beluga_estimation::Ref<beluga_estimation::MatrixX<T>> measurement_covariance_block =
          state.workspace.measurement_covariance.topLeftCorner(residual_size, residual_size);

      const auto conditioning_state_tangent_size = static_cast<Eigen::Index>(
          state.structure.manifold->TangentSize(state.workspace.conditioning_variable_indices));
      if (state.workspace.measurement_jacobian.rows() < residual_size ||
          state.workspace.measurement_jacobian.cols() < conditioning_state_tangent_size) {
        state.workspace.measurement_jacobian.resize(residual_size, conditioning_state_tangent_size);
      }
      beluga_estimation::Ref<beluga_estimation::MatrixX<T>> measurement_jacobian_block =
          state.workspace.measurement_jacobian.topLeftCorner(residual_size, conditioning_state_tangent_size);

      normal_measurement_constraint->CalcResidual(
          {}, conditioning_state_values, to_view(measurement_residual_block), to_view(measurement_jacobian_block));

      normal_measurement_constraint->CalcCovariance(conditioning_state_values, to_view(measurement_covariance_block));

      middleware_->Debug(
          "ExtendedKalmanFilter: measurement constraint '{}' has residual {} and covariance diag {}", constraint->Id(),
          VectorLike(measurement_residual_block), MatrixLike(measurement_covariance_block, MatrixStyle::kDiagonal));

      const auto state_tangent_size = static_cast<Eigen::Index>(state.structure.manifold->TangentSize());
      if (state.workspace.full_measurement_jacobian.rows() < residual_size ||
          state.workspace.full_measurement_jacobian.cols() < state_tangent_size) {
        state.workspace.full_measurement_jacobian.resize(residual_size, state_tangent_size);
      }
      beluga_estimation::Ref<beluga_estimation::MatrixX<T>> full_measurement_jacobian_block =
          state.workspace.full_measurement_jacobian.topLeftCorner(residual_size, state_tangent_size);
      full_measurement_jacobian_block.setZero();
      state.structure.manifold->LiftTangent(
          to_view(measurement_jacobian_block), state.workspace.conditioning_variable_indices,
          to_view(full_measurement_jacobian_block), kColsAxis);

      // Innovation covariance combines projected state uncertainty and sensor noise.
      state.workspace.innovation_covariance =
          full_measurement_jacobian_block * state.estimate.covariance * full_measurement_jacobian_block.transpose() +
          measurement_covariance_block;

      middleware_->Debug(
          "ExtendedKalmanFilter: measurement constraint '{}' has innovation covariance diag {}", constraint->Id(),
          MatrixLike(state.workspace.innovation_covariance, MatrixStyle::kDiagonal));

      if (state.workspace.kalman_gain.rows() < state_tangent_size ||
          state.workspace.kalman_gain.cols() < residual_size) {
        state.workspace.kalman_gain.resize(state_tangent_size, residual_size);
      }
      beluga_estimation::Ref<beluga_estimation::MatrixX<T>> kalman_gain_block =
          state.workspace.kalman_gain.topLeftCorner(state_tangent_size, residual_size);

      // Solve instead of forming an explicit inverse for better numerical behavior.
      // That is, instead of computing K = P- * H^T * (H * P- * H^T + R)^-1 directly,
      // we solve the linear system (H * P- * H^T + R) * K^T = H * P- for K, which
      // is equivalent because both prior state and innovation covariance are symmetric.
      // Cholesky (or LDLT) decomposition for symmetric positive definite matrices
      // is more efficient and numerically stable than computing an inverse explicitly
      // for an arbitrary matrix.
      kalman_gain_block = state.workspace.innovation_covariance.ldlt()
                              .solve(full_measurement_jacobian_block * state.estimate.covariance)
                              .transpose();
      state.workspace.correction_term = kalman_gain_block * measurement_residual_block;
      state.structure.manifold->Plus(
          to_view(state.estimate.mean), to_view(state.workspace.correction_term), to_view(state.estimate.mean));

      middleware_->Debug(
          "ExtendedKalmanFilter: measurement constraint '{}' applied a correction of {}", constraint->Id(),
          VectorLike(state.workspace.correction_term));

      state.workspace.update_multiplier =
          state.workspace.identity - kalman_gain_block * full_measurement_jacobian_block;

      // The Joseph or stabilized form of the covariance update can be shown equivalent to
      // the standard covariance update P+ = (I - K * H) * P- while less sensitive to round
      // off errors (to first order), helping preserve the symmetry and positive semidefiniteness
      // of the covariance matrix. For further reference, see section 3.3.2, Crassidis, J. L., &
      // Junkins, J. L. (2011). Optimal Estimation of Dynamic Systems (2nd ed.). CRC Press.
      state.estimate.covariance.noalias() =
          (state.workspace.update_multiplier * state.estimate.covariance *
               state.workspace.update_multiplier.transpose() +
           kalman_gain_block * measurement_covariance_block * kalman_gain_block.transpose());
    }

    return Ok();
  }

  /// Middleware handle used to report diagnostics.
  std::shared_ptr<MiddlewareInterface> middleware_;

  /// Immutable parameter-derived setup for this solver instance.
  Configuration config_;

  /// Stateful posterior and workspace; reset by @ref Finalize.
  std::unique_ptr<State> state_;
};

using ExtendedKalmanFilter = ExtendedKalmanFilter_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation::ExtendedKalmanFilter, beluga_estimation::BayesianOperation);
