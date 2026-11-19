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

#include <memory>
#include <utility>

#include <range/v3/view/concat.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/reverse.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/take.hpp>

#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/compute/program.hpp>
#include <beluga_estimation/compute/utilities.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>

namespace beluga_estimation {

/**
 * @brief A recursive Bayesian filter program that executes staged operations in a loop.
 *
 * `RecursiveFilter` implements a general-purpose Bayesian estimation loop consisting of:
 * - **Initialization stages**: Run once at startup to set up the filter state.
 * - **Input stages**: Run each iteration to acquire new measurements or data.
 * - **Preprocessing stages**: Run each iteration to prepare data for the solver.
 * - **Solver stage**: The core estimation algorithm (e.g., particle filter update).
 * - **Output stages**: Run each iteration to publish results or diagnostics.
 *
 * The filter runs at a configurable rate and respects the provided compute budget,
 * returning early if interrupted or a timeout occurs.
 *
 * Execution progress — stage initialization, each loop iteration, busy/skipped
 * stages, and failures — is reported at debug level through the middleware logger,
 * prefixed with `RecursiveFilter: `.
 *
 * All stages are instantiated from parameters using the registry system, allowing
 * runtime configuration of the filter pipeline.
 *
 * A runnable pipeline requires at least one input stage and a solver stage: with
 * no input stage nothing new enters the network, so the solver would only
 * re-evaluate the state it was initialized with. @ref Run rejects such a
 * configuration with `Error::kFailure` before any stage is initialized.
 *
 * @tparam T Scalar type used by the filter (e.g., `float` or `double`).
 */
template <typename T>
class RecursiveFilter_ : public BayesianProgram_<T> {
 public:
  /**
   * @brief Configuration structure for the recursive filter.
   *
   * Specifies all processing stages and the execution rate for the filter.
   */
  struct Configuration {
    /**
     * @brief Loads filter configuration from middleware parameters.
     *
     * Expects the following parameters under the given prefix:
     * - `<prefix>init.stages`: List of stage names for initialization
     * - `<prefix>input.stages`: List of stage names for input acquisition
     * - `<prefix>preprocess.stages`: List of stage names for preprocessing (optional, defaults to none)
     * - `<prefix>solver.type`: Type name for the solver stage
     * - `<prefix>output.stages`: List of stage names for output publishing
     * - `<prefix>rate`: Loop execution rate in Hz (default: 10.0)
     *
     * Each stage is instantiated via the registry using its type parameter.
     * At least one input stage and a solver stage must be configured; @ref Run
     * rejects the configuration otherwise.
     *
     * @param middleware The middleware interface for parameter access.
     * @param prefix The parameter namespace prefix (e.g., "filter.").
     * @return A result containing the configuration, or an error if any stage type is not registered.
     */
    static Result_<Configuration> LoadFromParams(
        std::shared_ptr<MiddlewareInterface> middleware,
        const std::string& prefix = "") {
      Configuration config;

      auto init_result = MakeInstancesFromParams<BayesianOperation_<T>>(
          middleware, middleware->DeclareParameter<std::vector<std::string>>(prefix + "init.stages"), prefix + "init.");
      if (!init_result) {
        return Err(CollectError(init_result));
      }
      config.init_stages = Unwrap(init_result);

      auto input_result = MakeInstancesFromParams<BayesianOperation_<T>>(
          middleware, middleware->DeclareParameter<std::vector<std::string>>(prefix + "input.stages"),
          prefix + "input.");
      if (!input_result) {
        return Err(CollectError(input_result));
      }
      config.input_stages = Unwrap(input_result);

      auto preprocess_result = MakeInstancesFromParams<BayesianOperation_<T>>(
          middleware, middleware->DeclareParameter<std::vector<std::string>>(prefix + "preprocess.stages", {}),
          prefix + "preprocess.");
      if (!preprocess_result) {
        return Err(CollectError(preprocess_result));
      }
      config.preprocess_stages = Unwrap(preprocess_result);

      auto solver_result = MakeInstanceFromParams<BayesianOperation_<T>>(middleware, prefix + "solver.");
      if (!solver_result) {
        return Err(CollectError(solver_result));
      }
      config.solver_stage = Unwrap(solver_result);

      auto output_result = MakeInstancesFromParams<BayesianOperation_<T>>(
          middleware, middleware->DeclareParameter<std::vector<std::string>>(prefix + "output.stages"),
          prefix + "output.");
      if (!output_result) {
        return Err(CollectError(output_result));
      }
      config.output_stages = Unwrap(output_result);

      config.rate = middleware->DeclareParameter<double>(prefix + "rate");
      return config;
    }

    /// Stages executed once during initialization.
    std::vector<std::shared_ptr<BayesianOperation_<T>>> init_stages;
    /// Stages executed each iteration to acquire input data.
    std::vector<std::shared_ptr<BayesianOperation_<T>>> input_stages;
    /// Stages executed each iteration to preprocess data.
    std::vector<std::shared_ptr<BayesianOperation_<T>>> preprocess_stages;
    /// The core solver stage executed each iteration.
    std::shared_ptr<BayesianOperation_<T>> solver_stage;
    /// Stages executed each iteration to publish outputs.
    std::vector<std::shared_ptr<BayesianOperation_<T>>> output_stages;
    /// Loop execution rate in Hz.
    double rate;
  };

  /**
   * @brief Creates a recursive filter from middleware parameters.
   *
   * Loads the configuration and creates the filter, or returns an error if
   * configuration loading fails (e.g., a stage type is not registered).
   *
   * @param middleware The middleware interface for parameter access and dependency injection.
   * @param prefix The parameter namespace prefix for this filter (e.g., "filter.").
   * @return A result containing the filter, or an error if configuration fails.
   */
  static Result_<std::unique_ptr<RecursiveFilter_<T>>> Make(
      std::shared_ptr<MiddlewareInterface> middleware,
      std::string_view prefix) {
    auto config_result = Configuration::LoadFromParams(middleware, std::string(prefix));
    if (!config_result) {
      return Err(CollectError(config_result));
    }
    return std::make_unique<RecursiveFilter_<T>>(std::move(middleware), Unwrap(config_result));
  }

  /**
   * @brief Constructs a recursive filter from middleware parameters.
   *
   * Loads the configuration using `Configuration::LoadFromParams()` and
   * initializes the filter.
   *
   * @param middleware The middleware interface for parameter access and dependency injection.
   * @param prefix The parameter namespace prefix for this filter (e.g., "filter.").
   * @throws std::runtime_error if any stage type is not found in the registry.
   */
  explicit RecursiveFilter_(std::shared_ptr<MiddlewareInterface> middleware, std::string_view prefix)
      : RecursiveFilter_(middleware, Unwrap(Configuration::LoadFromParams(middleware, std::string(prefix)))) {}

  /**
   * @brief Constructs a recursive filter from an explicit configuration.
   *
   * @param middleware The middleware interface for clock and rate utilities.
   * @param config The filter configuration specifying all stages and execution rate.
   */
  explicit RecursiveFilter_(std::shared_ptr<MiddlewareInterface> middleware, Configuration config)
      : middleware_(std::move(middleware)), context_(std::make_unique<Context_<T>>()), config_(std::move(config)) {}

  /**
   * @brief Runs the recursive filter loop until completion or budget exhaustion.
   *
   * Execution proceeds in the following phases:
   * 1. **Initialize**: Calls `Initialize()` on all stages in sequence.
   * 2. **Main loop**: Executes at the configured rate:
   *    - Runs init stages
   *    - Loops until budget is exhausted over:
   *      - Input stages (skips preprocessing if BUSY)
   *      - Preprocess stages
   *      - Solver stage (runs on the first iteration even if inputs are BUSY,
   *        so it can bootstrap from the initialized state)
   *      - Output stages
   * 3. **Finalize**: Calls `Finalize()` on all stages in reverse order.
   *
   * The loop continues until:
   * - The compute budget is exhausted (returns an error with `Error::kTimeout`
   *   or `Error::kInterrupted`)
   * - Any stage returns an error result
   *
   * @param budget Resource limits (deadline and/or stop token) for this run.
   * @return The resulting result (success, or an error with
   *         `Error::kTimeout`/`kInterrupted`/`kFailure`). A configuration
   *         without input stages or without a solver stage is rejected with
   *         `Error::kFailure` before any stage is initialized.
   */
  Result Run(const ComputeBudget& budget) override {
    Result outcome;

    if (auto status = ValidateConfiguration(); !status) {
      return status;
    }

    auto all_stages = ranges::views::concat(
        config_.init_stages, config_.input_stages, config_.preprocess_stages,
        ranges::views::single(config_.solver_stage), config_.output_stages);

    const std::size_t num_stages = config_.init_stages.size() + config_.input_stages.size() +
                                   config_.preprocess_stages.size() + (config_.solver_stage ? 1U : 0U) +
                                   config_.output_stages.size();
    middleware_->Debug(
        "RecursiveFilter: starting run ({} init, {} input, {} preprocess, {} solver, {} output stages)",
        config_.init_stages.size(), config_.input_stages.size(), config_.preprocess_stages.size(),
        config_.solver_stage ? 1U : 0U, config_.output_stages.size());

    std::size_t num_initialized = 0;
    for (auto&& [index, stage] : ranges::views::zip(ranges::views::iota(1u), all_stages)) {
      middleware_->Debug("RecursiveFilter: initializing stage {}/{}", index, num_stages);
      outcome = stage->Initialize(budget);
      if (!outcome) {
        // Initialization is all or nothing.
        // If any stage fails to completely initialize,
        // we skip the rest, skip the run, and go
        // straight to finalization.
        middleware_->Debug("RecursiveFilter: stage {}/{} failed to initialize: {}", index, num_stages, outcome.error());
        break;
      }
      ++num_initialized;
    }
    middleware_->Debug("RecursiveFilter: {}/{} stages initialized", num_initialized, num_stages);

    if (outcome) {
      outcome = DoRun(budget);
    }

    std::vector<Error> side_effects;
    auto initialized_stages = ranges::views::take(all_stages, num_initialized);
    for (auto&& [index, stage] : ranges::views::reverse(
             ranges::views::zip(ranges::views::iota(1u, num_initialized + 1u), initialized_stages))) {
      middleware_->Debug("RecursiveFilter: finalizing stage {}/{}", index, num_initialized);
      // Cleanup failures are recorded as side effects of the run outcome rather
      // than replacing it, so the caller still sees the original failure code.
      auto finalize_outcome = stage->Finalize(budget);
      if (!finalize_outcome) {
        middleware_->Debug(
            "RecursiveFilter: stage {}/{} failed to finalize: {}", index, num_initialized, finalize_outcome.error());
        side_effects.push_back(CollectError(finalize_outcome));
      }
    }

    if (outcome) {
      if (side_effects.empty()) {
        middleware_->Debug("RecursiveFilter: run finished successfully");
        return outcome;
      }
      // The run succeeded but cleanup failed: report a fresh failure carrying
      // the cleanup failures as side effects.
      middleware_->Debug("RecursiveFilter: run finished, but {} stage(s) failed to finalize", side_effects.size());
      return Err(Error::kFailure, "stage finalization failed", std::move(side_effects));
    }

    // The run failed: keep it as the head and record cleanup failures as its
    // side effects.
    middleware_->Debug(
        "RecursiveFilter: run failed: {} ({} finalization failure(s))", outcome.error(), side_effects.size());
    return Err(CollectError(outcome), std::move(side_effects));
  }

 private:
  /**
   * @brief Rejects configurations that cannot produce a meaningful run.
   *
   * Input stages are what feed the network each iteration; without them the
   * solver would keep re-evaluating the state it was initialized with. The
   * solver stage is what advances the estimate, so it is equally mandatory.
   *
   * @return Success, or an error with `Error::kFailure` for an unusable configuration.
   */
  Result ValidateConfiguration() {
    if (config_.input_stages.empty()) {
      middleware_->Error("RecursiveFilter: at least one input stage is required to feed the filter");
      return Err(Error::kFailure, "at least one input stage is required");
    }
    if (!config_.solver_stage) {
      middleware_->Error("RecursiveFilter: a solver stage is required to advance the estimate");
      return Err(Error::kFailure, "a solver stage is required");
    }
    return Ok();
  }

  Result DoRun(const ComputeBudget& budget) {
    const size_t num_init_stages = config_.init_stages.size();
    middleware_->Debug("RecursiveFilter: executing {} init stage(s)", num_init_stages);
    for (auto&& [index, stage] : ranges::views::zip(ranges::views::iota(1u), config_.init_stages)) {
      middleware_->Debug("RecursiveFilter: executing init stage {}/{}", index, num_init_stages);
      Result stage_outcome = stage->Execute(*context_, budget);
      if (!stage_outcome) {
        // Filter initialization is all or nothing too.
        // If any stage fails to complete we skip the run.
        middleware_->Debug("RecursiveFilter: init stage {} failed: {}", index, stage_outcome.error());
        return stage_outcome;
      }
    }

    Result loop_outcome;
    std::size_t iteration = 0;
    auto clock = middleware_->GetClockInterface();
    auto rate = middleware_->CreateRate(config_.rate);
    while ((loop_outcome = budget.Check(clock->Now())) == Ok()) {
      middleware_->Debug("RecursiveFilter: iteration {} started", iteration);

      const size_t num_input_stages = config_.input_stages.size();
      // Input stages are guaranteed to exist (see ValidateConfiguration), so
      // assume they are busy until at least one of them succeeds.
      bool skip_preprocess = true;
      for (auto&& [index, stage] : ranges::views::zip(ranges::views::iota(1u), config_.input_stages)) {
        middleware_->Debug("RecursiveFilter: executing input stage {}/{}", index, num_input_stages);
        Result stage_outcome = stage->Execute(*context_, budget);
        if (!stage_outcome) {
          if (stage_outcome.error() == Error::kBusy) {
            middleware_->Debug("RecursiveFilter: input stage {}/{} is busy", index, num_input_stages);
            continue;
          }
          middleware_->Debug(
              "RecursiveFilter: input stage {}/{} failed: {}", index, num_input_stages, stage_outcome.error());
          return stage_outcome;
        }
        // If at least one input stage is successful, we preprocess.
        skip_preprocess = false;
      }
      bool skip_solver = skip_preprocess;
      if (!skip_preprocess) {
        const size_t num_preprocess_stages = config_.preprocess_stages.size();
        for (auto&& [index, stage] : ranges::views::zip(ranges::views::iota(1u), config_.preprocess_stages)) {
          middleware_->Debug("RecursiveFilter: executing preprocess stage {}/{}", index, num_preprocess_stages);
          Result stage_outcome = stage->Execute(*context_, budget);
          if (!stage_outcome) {
            if (stage_outcome.error() == Error::kBusy) {
              // A busy preprocessing stage has no fresh data for the solver.
              middleware_->Debug("RecursiveFilter: preprocess stage {}/{} is busy", index, num_preprocess_stages);
              skip_solver = true;
              break;
            }
            middleware_->Debug(
                "RecursiveFilter: preprocess stage {}/{} failed: {}", index, num_preprocess_stages,
                stage_outcome.error());
            return stage_outcome;
          }
        }
      } else {
        middleware_->Debug("RecursiveFilter: skipping preprocess stages due to busy input stage(s)");
      }
      if (iteration == 0) {
        middleware_->Debug("RecursiveFilter: first iteration, solving from initialized state anyways");
        skip_solver = false;
      }
      if (!skip_solver) {
        middleware_->Debug("RecursiveFilter: executing solver stage");
        Result stage_outcome = config_.solver_stage->Execute(*context_, budget);
        if (!stage_outcome && stage_outcome.error() != Error::kBusy) {
          middleware_->Debug("RecursiveFilter: solver stage failed: {}", stage_outcome.error());
          return stage_outcome;
        }
        if (!stage_outcome) {
          middleware_->Debug("RecursiveFilter: solver stage is busy");
        }
      } else {
        middleware_->Debug("RecursiveFilter: skipping solver stage due to busy input/preprocess stage(s)");
      }
      // Output stages keep their pace regardless of the loop status.
      const size_t num_output_stages = config_.output_stages.size();
      for (auto&& [index, stage] : ranges::views::zip(ranges::views::iota(1u), config_.output_stages)) {
        middleware_->Debug("RecursiveFilter: executing output stage {}/{}", index, num_output_stages);
        Result stage_outcome = stage->Execute(*context_, budget);
        if (!stage_outcome) {
          if (stage_outcome.error() == Error::kBusy) {
            // If an output stage is busy, we still execute the rest.
            middleware_->Debug("RecursiveFilter: output stage {}/{} is busy", index, num_output_stages);
            continue;
          }
          middleware_->Debug(
              "RecursiveFilter: output stage {}/{} failed: {}", index, num_output_stages, stage_outcome.error());
          return stage_outcome;
        }
      }
      middleware_->Debug("RecursiveFilter: iteration {} completed", iteration);
      rate->Sleep(budget);
      ++iteration;
    }
    middleware_->Debug("RecursiveFilter: main loop exited after {} iteration(s): {}", iteration, loop_outcome.error());
    return loop_outcome;
  }

  std::shared_ptr<MiddlewareInterface> middleware_;
  std::unique_ptr<Context_<T>> context_;
  Configuration config_;
};

using RecursiveFilter = RecursiveFilter_<DefaultScalar>;

}  // namespace beluga_estimation

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation::RecursiveFilter, beluga_estimation::BayesianProgram);
