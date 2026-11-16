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
 * @brief Defines the VariablePriorInitializer operation.
 */

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/probability.hpp>
#include <beluga_estimation/common/registry.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/constraints/normal_prior_constraint.hpp>
#include <beluga_estimation/constraints/se2_normal_pose_prior_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace beluga_estimation {

/**
 * @brief Sets normal priors for one or more variables from parameters.
 *
 * On the first @ref Execute call, this operation creates, at the current time,
 * a variable and its normal prior constraint for each configured variable
 * (either SE(2) or Euclidean), and reports success. Subsequent calls are
 * no-ops that also report success. @ref Finalize resets the operation so
 * the next @ref Execute call re-initializes the variables at a new timestamp.
 *
 * The parameter schema, under the operation prefix, is:
 * - `<prefix>variables`: list of variable names.
 * - For each name `v` in that list:
 *   - `<prefix><v>.type`: `"se2"` or `"euclidean"`.
 *   - `<prefix><v>.mean`: mean as a double array.
 *     For `"se2"` the mean is in pose form `[x, y, theta]`; for
 *     `"euclidean"` it is the plain vector mean.
 *   - `<prefix><v>.covariance`: covariance in row-major vector form.
 *     For `"se2"` this is the 3x3 tangent-space covariance; for
 *     `"euclidean"` the n x n covariance of the variable.
 *
 * The Euclidean dimension is implied by the mean and covariance sizes and must
 * be consistent: a mean of size `n > 0` requires a covariance of size `n * n`.
 *
 * The configuration is validated during @ref Initialize; an empty
 * configuration or any invalid specification fails initialization with
 * `Error::kFailure` before any execution takes place.
 *
 * @tparam T Scalar type (e.g. `float` or `double`).
 */
template <typename T>
class VariablePriorInitializer_ : public BayesianOperation_<T> {
 public:
  /// @brief Configuration for the prior initializer.
  struct Configuration {
    /// @brief Parameter specification for a single prior variable.
    struct VariableSpec {
      std::string name;           ///< Name of the variable.
      std::string type;           ///< Variable type: `"se2"` or `"euclidean"`.
      std::vector<T> mean;        ///< Mean (pose form `[x, y, theta]` for SE(2)).
      std::vector<T> covariance;  ///< Covariance in row-major vector form.
    };

    /**
     * @brief Loads configuration from middleware parameters.
     *
     * @param middleware The middleware interface for parameter access.
     * @param prefix The parameter namespace prefix.
     * @return A fully populated Configuration instance.
     */
    static Configuration LoadFromParams(
        std::shared_ptr<MiddlewareInterface> middleware,
        const std::string& prefix = "") {
      Configuration config;
      const auto names = middleware->DeclareParameter<std::vector<std::string>>(prefix + "variables");
      for (const auto& name : names) {
        auto& spec = config.specs.emplace_back();
        spec.name = name;
        spec.type = middleware->DeclareParameter<std::string>(prefix + name + ".type");
        auto mean_value = middleware->DeclareParameter<std::vector<double>>(prefix + name + ".mean");
        auto covariance_value = middleware->DeclareParameter<std::vector<double>>(prefix + name + ".covariance");
        spec.mean.assign(mean_value.begin(), mean_value.end());
        spec.covariance.assign(covariance_value.begin(), covariance_value.end());
      }
      return config;
    }

    /// Ordered list of variable specifications to initialize.
    std::vector<VariableSpec> specs;
  };

  /**
   * @brief Creates a prior initializer from middleware parameters.
   *
   * @param middleware The middleware interface.
   * @param prefix The parameter namespace prefix for this operation.
   * @return A result containing the initializer instance.
   */
  static Result_<std::unique_ptr<VariablePriorInitializer_<T>>> Make(
      std::shared_ptr<MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<VariablePriorInitializer_<T>>(std::move(middleware), prefix);
  }

  /**
   * @brief Constructs the prior initializer from middleware parameters.
   * @param middleware The middleware interface.
   * @param prefix The parameter namespace prefix for this operation.
   */
  explicit VariablePriorInitializer_(std::shared_ptr<MiddlewareInterface> middleware, std::string_view prefix)
      : VariablePriorInitializer_(middleware, Configuration::LoadFromParams(middleware, std::string(prefix))) {}

  /**
   * @brief Constructs the prior initializer from an explicit configuration.
   * @param middleware The middleware interface.
   * @param config The operation configuration.
   */
  explicit VariablePriorInitializer_(std::shared_ptr<MiddlewareInterface> middleware, Configuration config)
      : middleware_(std::move(middleware)), config_(std::move(config)) {}

  /**
   * @brief Validates the configuration.
   *
   * Fails with `Error::kFailure` if the configuration has no variable
   * specifications or if any specification is invalid (empty or duplicate
   * names, unknown type, inconsistent mean/covariance sizes, or a covariance
   * that is not symmetric and positive definite). This is the all-or-nothing
   * validation gate for the filter's initialization phase.
   *
   * @param budget Resource limits for initialization.
   * @return Success, or an error with `Error::kFailure` on invalid configuration.
   */
  Result Initialize(const ComputeBudget& /*budget*/) override { return ValidateConfiguration(); }

  /**
   * @brief Sets the configured variables and prior constraints at the current time.
   *
   * Only the first call performs any work; subsequent calls are no-ops. The
   * configuration is assumed to have been validated by @ref Initialize. The
   * call fails with `Error::kFailure` if a variable or constraint cannot be
   * added to the network (e.g., a same-named variable already exists at the
   * current timestamp).
   *
   * @param context Execution context with DBN and estimate store.
   * @param budget Resource limits for this invocation.
   * @return Success (including no-op calls), or an error with `Error::kFailure` on failure to add.
   */
  Result Execute(Context_<T>& context, const ComputeBudget& /*budget*/) override {
    // This operation performs a bounded amount of work (one variable and one
    // constraint per configured name) and never loops, so the budget is not
    // consulted.
    if (executed_once_) {
      return Ok();
    }

    // All variables share a single timestamp captured once.
    const auto now = middleware_->GetClockInterface()->Now();

    for (const auto& spec : config_.specs) {
      Result status = AddVariableAndPrior(context, spec, now);
      if (!status) {
        return Err(CollectError(status));
      }
    }

    executed_once_ = true;
    middleware_->Debug("VariablePriorInitializer: initialized {} variable prior(s)", config_.specs.size());
    return Ok();
  }

  /**
   * @brief Releases resources and resets the one-shot state.
   * @param budget Resource limits for finalization.
   * @return Success.
   */
  Result Finalize(const ComputeBudget& /*budget*/) override {
    executed_once_ = false;
    return Ok();
  }

 private:
  /// @brief Alias for the per-variable configuration type.
  using VariableSpec = typename Configuration::VariableSpec;

  /// @brief Validates every configured variable specification.
  Result ValidateConfiguration() const {
    // A configuration with no variable specifications is a configuration error.
    if (config_.specs.empty()) {
      middleware_->Error("VariablePriorInitializer: no variable specifications were configured.");
      return Err(Error::kFailure, "no variable specifications were configured");
    }

    std::unordered_set<std::string> names;
    names.reserve(config_.specs.size());
    for (const auto& spec : config_.specs) {
      if (spec.name.empty()) {
        middleware_->Error("VariablePriorInitializer: variable name must not be empty.");
        return Err(Error::kFailure, "variable name must not be empty");
      }
      if (!names.insert(spec.name).second) {
        middleware_->Error(
            "VariablePriorInitializer: variable name '{}' is listed more than once in the configuration.", spec.name);
        return Err(Error::kFailure, fmt::format("variable name '{}' is listed more than once", spec.name));
      }

      if (spec.type == "se2") {
        if (spec.mean.size() != 3) {
          middleware_->Error(
              "VariablePriorInitializer: SE(2) mean for variable '{}' must have size 3 (pose form [x, y, theta]).",
              spec.name);
          return Err(Error::kFailure, fmt::format("SE(2) mean for variable '{}' must have size 3", spec.name));
        }
        if (spec.covariance.size() != 9) {
          middleware_->Error(
              "VariablePriorInitializer: SE(2) covariance for variable '{}' must have size 9 (3x3 row-major).",
              spec.name);
          return Err(Error::kFailure, fmt::format("SE(2) covariance for variable '{}' must have size 9", spec.name));
        }
        if (!IsCovarianceMatrix(beluga_estimation::Map<const beluga_estimation::Matrix3<T>>(spec.covariance.data()))) {
          middleware_->Error(
              "VariablePriorInitializer: covariance for variable '{}' must be a valid covariance matrix (symmetric "
              "and positive definite).",
              spec.name);
          return Err(Error::kFailure, fmt::format("covariance for variable '{}' is not positive definite", spec.name));
        }
      } else if (spec.type == "euclidean") {
        const std::size_t ndim = spec.mean.size();
        if (ndim == 0) {
          middleware_->Error(
              "VariablePriorInitializer: Euclidean mean for variable '{}' must not be empty.", spec.name);
          return Err(Error::kFailure, fmt::format("Euclidean mean for variable '{}' must not be empty", spec.name));
        }
        if (spec.covariance.size() != ndim * ndim) {
          middleware_->Error(
              "VariablePriorInitializer: Euclidean covariance for variable '{}' must have size {} (n x n row-major).",
              spec.name, ndim * ndim);
          return Err(
              Error::kFailure, fmt::format("Euclidean covariance for variable '{}' has the wrong size", spec.name));
        }
        if (!IsCovarianceMatrix(
                beluga_estimation::Map<const beluga_estimation::MatrixX<T>>(spec.covariance.data(), ndim, ndim))) {
          middleware_->Error(
              "VariablePriorInitializer: covariance for variable '{}' must be a valid covariance matrix (symmetric "
              "and positive definite).",
              spec.name);
          return Err(Error::kFailure, fmt::format("covariance for variable '{}' is not positive definite", spec.name));
        }
      } else {
        middleware_->Error("VariablePriorInitializer: unknown type '{}' for variable '{}'.", spec.type, spec.name);
        return Err(Error::kFailure, fmt::format("unknown type '{}' for variable '{}'", spec.type, spec.name));
      }
    }
    return Ok();
  }

  /// @brief Adds a variable and its prior constraint for a single specification.
  Result AddVariableAndPrior(Context_<T>& context, const VariableSpec& spec, TimePoint now) const {
    if (spec.type == "se2") {
      return AddSE2VariableAndPrior(context, spec, now);
    }
    if (spec.type == "euclidean") {
      return AddEuclideanVariableAndPrior(context, spec, now);
    }
    middleware_->Error("VariablePriorInitializer: unknown type '{}' for variable '{}'.", spec.type, spec.name);
    return Err(Error::kFailure, fmt::format("unknown type '{}' for variable '{}'", spec.type, spec.name));
  }

  /// @brief Adds an SE(2) variable and its normal prior constraint.
  Result AddSE2VariableAndPrior(Context_<T>& context, const VariableSpec& spec, TimePoint now) const {
    const auto mean = beluga_estimation::Map<const beluga_estimation::Vector3<T>>(spec.mean.data());
    const beluga_estimation::SE2<T> pose(
        beluga_estimation::SO2<T>::exp(mean(2)), beluga_estimation::Vector2<T>{mean(0), mean(1)});
    const auto covariance = beluga_estimation::Map<const beluga_estimation::Matrix3<T>>(spec.covariance.data());

    const Identifier variable_id{spec.name, now};
    const Identifier constraint_id{spec.name + "/prior", now};

    auto variable = std::make_shared<SE2Variable_<T>>(variable_id);
    if (auto added = context.dbn.Add(std::move(variable)); !added) {
      middleware_->Error("VariablePriorInitializer: variable '{}' already exists in the DBN.", spec.name);
      return Err(CollectError(added));
    }
    auto constraint = std::make_shared<SE2NormalPosePriorConstraint_<T>>(
        constraint_id, variable_id, SE2NormalDistribution_<T>(pose, covariance));
    if (auto added = context.dbn.Add(std::move(constraint)); !added) {
      middleware_->Error(
          "VariablePriorInitializer: prior constraint for variable '{}' already exists in the DBN.", spec.name);
      return Err(CollectError(added));
    }
    return Ok();
  }

  /// @brief Adds a Euclidean variable (runtime dimension) and its normal prior constraint.
  Result AddEuclideanVariableAndPrior(Context_<T>& context, const VariableSpec& spec, TimePoint now) const {
    const std::size_t ndim = spec.mean.size();
    const auto mean = beluga_estimation::Map<const beluga_estimation::VectorX<T>>(spec.mean.data(), ndim);
    const auto covariance =
        beluga_estimation::Map<const beluga_estimation::MatrixX<T>>(spec.covariance.data(), ndim, ndim);

    const Identifier variable_id{spec.name, now};
    const Identifier constraint_id{spec.name + "/prior", now};

    auto variable = std::make_shared<EuclideanVariable_<T>>(variable_id, ndim);
    if (auto added = context.dbn.Add(std::move(variable)); !added) {
      middleware_->Error("VariablePriorInitializer: variable '{}' already exists in the DBN.", spec.name);
      return Err(CollectError(added));
    }
    auto distribution = std::make_shared<NormalDistribution_<T>>(mean, covariance);
    auto constraint = std::make_shared<NormalPriorConstraint_<T>>(constraint_id, variable_id, std::move(distribution));
    if (auto added = context.dbn.Add(std::move(constraint)); !added) {
      middleware_->Error(
          "VariablePriorInitializer: prior constraint for variable '{}' already exists in the DBN.", spec.name);
      return Err(CollectError(added));
    }
    return Ok();
  }

  std::shared_ptr<MiddlewareInterface> middleware_;
  Configuration config_;
  bool executed_once_{false};
};

using VariablePriorInitializer = VariablePriorInitializer_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation::VariablePriorInitializer, beluga_estimation::BayesianOperation);
