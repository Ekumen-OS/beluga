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
 * @brief Includes Bayesian operation mock for testing.
 */

#include <string>
#include <utility>

#include <gmock/gmock.h>

#include <beluga_estimation/compute/operation.hpp>

namespace beluga_estimation {

/**
 * @brief Mock implementation of BayesianOperation_ for testing.
 *
 * Provides a controllable operation that tracks lifecycle calls (Initialize,
 * Execute, Finalize) and allows configuring return result values.
 *
 * @tparam T Scalar type used by the Bayesian network.
 */
template <typename T>
class MockBayesianOperation_ : public BayesianOperation_<T> {
 public:
  /**
   * @brief Constructs a mock operation with an optional name.
   * @param name Human-readable name for debugging.
   */
  explicit MockBayesianOperation_(std::string name = "mock_op") : name_(std::move(name)) {}

  Result Initialize(const ComputeBudget& /*budget*/) override {
    init_called_ = true;
    return std::move(init_result_);
  }

  Result Execute(Context_<T>& /*context*/, const ComputeBudget& /*budget*/) override {
    exec_count_++;
    return std::move(exec_result_);
  }

  Result Finalize(const ComputeBudget& /*budget*/) override {
    finalize_called_ = true;
    return std::move(finalize_result_);
  }

  /// Returns true if Initialize() was called.
  bool InitCalled() const { return init_called_; }

  /// Returns true if Finalize() was called.
  bool FinalizeCalled() const { return finalize_called_; }

  /// Returns the number of times Execute() was called.
  int ExecCount() const { return exec_count_; }

  /// Returns the operation name.
  const std::string& Name() const { return name_; }

  /// Sets the result to be returned by Initialize().
  void SetInitResult(Result result) { init_result_ = std::move(result); }

  /// Sets the result to be returned by Execute().
  void SetExecResult(Result result) { exec_result_ = std::move(result); }

  /// Sets the result to be returned by Finalize().
  void SetFinalizeResult(Result result) { finalize_result_ = std::move(result); }

 private:
  std::string name_;
  bool init_called_ = false;
  bool finalize_called_ = false;
  int exec_count_ = 0;
  Result init_result_;
  Result exec_result_;
  Result finalize_result_;
};

using MockBayesianOperation = MockBayesianOperation_<DefaultScalar>;

}  // namespace beluga_estimation
