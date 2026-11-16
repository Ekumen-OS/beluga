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

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <beluga_estimation/compute/programs/recursive_filter.hpp>
#include <beluga_estimation/mocks/mock_clock.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/mocks/mock_operation.hpp>
#include <beluga_estimation/mocks/mock_rate.hpp>

namespace {

using namespace beluga_estimation;
using ::testing::_;
using ::testing::Return;

// Test fixture
class RecursiveFilterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    middleware_ = std::make_shared<MockMiddleware>();
    clock_ = std::make_shared<MockClock>();

    ON_CALL(*middleware_, GetClockInterface()).WillByDefault(Return(clock_.get()));
    EXPECT_CALL(*middleware_, GetClockInterface()).Times(::testing::AnyNumber());

    ON_CALL(*middleware_, CreateRate(_)).WillByDefault([this](double freq) {
      auto rate = std::make_unique<MockRate>(freq, clock_.get());
      last_rate_ = rate.get();
      return rate;
    });
    EXPECT_CALL(*middleware_, CreateRate(_)).Times(::testing::AnyNumber());

    ON_CALL(*middleware_->logger, Log(_, _)).WillByDefault(Return());
    EXPECT_CALL(*middleware_->logger, Log(_, _)).Times(::testing::AnyNumber());
  }

  // Adds a healthy input stage, since a runnable filter must be fed.
  static std::shared_ptr<MockBayesianOperation> AddInputStage(RecursiveFilter::Configuration& config) {
    auto input_op = std::make_shared<MockBayesianOperation>("input");
    config.input_stages.push_back(input_op);
    return input_op;
  }

  std::shared_ptr<MockMiddleware> middleware_;
  std::shared_ptr<MockClock> clock_;
  MockRate* last_rate_ = nullptr;
};

// Test basic construction with manual configuration
TEST_F(RecursiveFilterTest, BasicConstruction) {
  RecursiveFilter::Configuration config;

  // Create operations
  auto init_op = std::make_shared<MockBayesianOperation>("init");
  auto input_op = std::make_shared<MockBayesianOperation>("input");
  auto preprocess_op = std::make_shared<MockBayesianOperation>("preprocess");
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");
  auto output_op = std::make_shared<MockBayesianOperation>("output");

  config.init_stages.push_back(init_op);
  config.input_stages.push_back(input_op);
  config.preprocess_stages.push_back(preprocess_op);
  config.solver_stage = solver_op;
  config.output_stages.push_back(output_op);
  config.rate = 10.0;

  // Construct filter
  RecursiveFilter filter(middleware_, std::move(config));

  SUCCEED();  // Just verify it constructs
}

// Test that all stages are initialized
TEST_F(RecursiveFilterTest, AllStagesInitialized) {
  RecursiveFilter::Configuration config;

  auto init_op = std::make_shared<MockBayesianOperation>("init");
  auto input_op = std::make_shared<MockBayesianOperation>("input");
  auto preprocess_op = std::make_shared<MockBayesianOperation>("preprocess");
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");
  auto output_op = std::make_shared<MockBayesianOperation>("output");

  config.init_stages.push_back(init_op);
  config.input_stages.push_back(input_op);
  config.preprocess_stages.push_back(preprocess_op);
  config.solver_stage = solver_op;
  config.output_stages.push_back(output_op);
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  // Use a very short deadline to terminate quickly after initialization
  auto deadline = clock_->Now() + std::chrono::milliseconds(1);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);

  // Verify all stages were initialized
  EXPECT_TRUE(init_op->InitCalled());
  EXPECT_TRUE(input_op->InitCalled());
  EXPECT_TRUE(preprocess_op->InitCalled());
  EXPECT_TRUE(solver_op->InitCalled());
  EXPECT_TRUE(output_op->InitCalled());
}

// Test that all stages are finalized
TEST_F(RecursiveFilterTest, AllStagesFinalized) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);

  auto init_op = std::make_shared<MockBayesianOperation>("init");
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");

  config.init_stages.push_back(init_op);
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  // Use stop token for immediate termination
  std_future::stop_source stop_source;
  stop_source.request_stop();
  ComputeBudget budget(std::nullopt, stop_source.get_token());

  Result status = filter.Run(budget);

  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kInterrupted);
  EXPECT_TRUE(init_op->FinalizeCalled());
  EXPECT_TRUE(solver_op->FinalizeCalled());
}

// Test that a cleanup failure is recorded as a side effect of the run outcome
TEST_F(RecursiveFilterTest, FinalizeFailureIsRecordedAsSideEffect) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);

  auto solver_op = std::make_shared<MockBayesianOperation>("solver");
  solver_op->SetFinalizeResult(Err(Error::kFailure, "finalize failed"));

  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  std_future::stop_source stop_source;
  stop_source.request_stop();
  ComputeBudget budget(std::nullopt, stop_source.get_token());

  Result status = filter.Run(budget);

  // The run outcome stays the head; the cleanup failure is a side effect.
  ASSERT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kInterrupted);
  ASSERT_EQ(status.error().GetSideEffects().size(), 1U);
  EXPECT_EQ(status.error().GetSideEffects()[0].GetDescription(), "finalize failed");
}

// Test that initialization failure prevents execution
TEST_F(RecursiveFilterTest, InitFailurePreventsExecution) {
  RecursiveFilter::Configuration config;

  auto init_op = std::make_shared<MockBayesianOperation>("init");
  auto input_op = std::make_shared<MockBayesianOperation>("input");
  input_op->SetInitResult(Err(Error::kFailure));  // Make initialization fail

  auto solver_op = std::make_shared<MockBayesianOperation>("solver");

  config.init_stages.push_back(init_op);
  config.input_stages.push_back(input_op);
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::seconds(10);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);

  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kFailure);
  EXPECT_EQ(input_op->ExecCount(), 0);   // Should not execute
  EXPECT_EQ(solver_op->ExecCount(), 0);  // Should not execute
}

// Test that execution runs the solver
TEST_F(RecursiveFilterTest, SolverExecutes) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);

  // Create a custom operation that stops after first exec
  struct StoppingSolver : public BayesianOperation_<double> {
    std_future::stop_source* stop_src;
    int& count;

    StoppingSolver(std_future::stop_source* src, int& c) : stop_src(src), count(c) {}

    Result Initialize(const ComputeBudget&) override { return Ok(); }
    Result Execute(Context_<double>& /*context*/, const ComputeBudget&) override {
      count++;
      if (count >= 1) {
        stop_src->request_stop();
      }
      return Ok();
    }
    Result Finalize(const ComputeBudget&) override { return Ok(); }
  };

  std_future::stop_source stop_source;
  int count = 0;
  auto stopping_solver = std::make_shared<StoppingSolver>(&stop_source, count);

  config.solver_stage = stopping_solver;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  ComputeBudget budget(std::nullopt, stop_source.get_token());
  Result status = filter.Run(budget);

  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kInterrupted);
  EXPECT_GE(count, 1);  // Solver should execute at least once
}

// Test that debug logs are emitted while running
TEST_F(RecursiveFilterTest, EmitsDebugLogsDuringRun) {
  RecursiveFilter::Configuration config;

  auto init_op = std::make_shared<MockBayesianOperation>("init");
  auto input_op = std::make_shared<MockBayesianOperation>("input");
  auto preprocess_op = std::make_shared<MockBayesianOperation>("preprocess");
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");
  auto output_op = std::make_shared<MockBayesianOperation>("output");

  config.init_stages.push_back(init_op);
  config.input_stages.push_back(input_op);
  config.preprocess_stages.push_back(preprocess_op);
  config.solver_stage = solver_op;
  config.output_stages.push_back(output_op);
  config.rate = 10.0;

  std::vector<std::string> messages;
  ON_CALL(*middleware_->logger, Log(LogLevel::kDebug, _))
      .WillByDefault([&messages](LogLevel, std::string_view message) { messages.emplace_back(message); });
  EXPECT_CALL(*middleware_->logger, Log(LogLevel::kDebug, _)).Times(::testing::AtLeast(1));

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(1);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);

  ASSERT_FALSE(messages.empty());
  for (const auto& message : messages) {
    EXPECT_EQ(message.rfind("RecursiveFilter: ", 0), 0);
  }

  const auto contains = [&messages](std::string_view needle) {
    return std::any_of(messages.begin(), messages.end(), [needle](const std::string& message) {
      return message.find(needle) != std::string::npos;
    });
  };
  EXPECT_TRUE(contains("starting run"));
  EXPECT_TRUE(contains("stages initialized"));
  EXPECT_TRUE(contains("executing init stage"));
  EXPECT_TRUE(contains("iteration 0 started"));
  EXPECT_TRUE(contains("iteration 0 completed"));
  EXPECT_TRUE(contains("main loop exited"));
  EXPECT_TRUE(contains("finalizing stage"));
  EXPECT_TRUE(contains("run failed"));
}

// Test with stop token (interruption)
TEST_F(RecursiveFilterTest, InterruptionViaStopToken) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);

  auto solver_op = std::make_shared<MockBayesianOperation>("solver");
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  // Create a stop token and request stop immediately
  std_future::stop_source stop_source;
  stop_source.request_stop();

  ComputeBudget budget(std::nullopt, stop_source.get_token());

  Result status = filter.Run(budget);
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kInterrupted);
}

// Test that init stages execute during run
TEST_F(RecursiveFilterTest, InitStagesExecute) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);

  auto init_op = std::make_shared<MockBayesianOperation>("init");
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");

  config.init_stages.push_back(init_op);
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(100);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);

  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);
  EXPECT_GE(init_op->ExecCount(), 1);  // Init should execute at least once
}

// Test with multiple operations per stage
TEST_F(RecursiveFilterTest, MultipleOperationsPerStage) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);

  // Add multiple init operations
  auto init_op1 = std::make_shared<MockBayesianOperation>("init1");
  auto init_op2 = std::make_shared<MockBayesianOperation>("init2");
  config.init_stages.push_back(init_op1);
  config.init_stages.push_back(init_op2);

  // Add solver
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(100);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);

  // All init ops should execute
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);
  EXPECT_GE(init_op1->ExecCount(), 1);
  EXPECT_GE(init_op2->ExecCount(), 1);
}

// Test BUSY status handling on input stages
TEST_F(RecursiveFilterTest, InputStageBusyHandling) {
  RecursiveFilter::Configuration config;

  auto input_op = std::make_shared<MockBayesianOperation>("input");
  input_op->SetExecResult(Err(Error::kBusy));  // Always return BUSY

  auto solver_op = std::make_shared<MockBayesianOperation>("solver");

  config.input_stages.push_back(input_op);
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(100);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);
  EXPECT_GE(input_op->ExecCount(), 1);
}

// Test that the solver bootstraps on the first iteration even when inputs are busy
TEST_F(RecursiveFilterTest, SolverRunsOnceWhenInputsBusyOnFirstIteration) {
  RecursiveFilter::Configuration config;

  auto init_op = std::make_shared<MockBayesianOperation>("init");
  auto input_op = std::make_shared<MockBayesianOperation>("input");
  input_op->SetExecResult(Err(Error::kBusy));  // Always return BUSY
  auto preprocess_op = std::make_shared<MockBayesianOperation>("preprocess");
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");

  config.init_stages.push_back(init_op);
  config.input_stages.push_back(input_op);
  config.preprocess_stages.push_back(preprocess_op);
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  // Long enough for several iterations at 10 Hz.
  auto deadline = clock_->Now() + std::chrono::milliseconds(350);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);

  EXPECT_GE(input_op->ExecCount(), 2);
  EXPECT_EQ(preprocess_op->ExecCount(), 0);
  // The solver runs once, right after initialization, and is skipped afterwards.
  EXPECT_EQ(solver_op->ExecCount(), 1);
}

// Test that a single successful input stage is enough to preprocess
TEST_F(RecursiveFilterTest, PreprocessRunsWhenAnyInputStageSucceeds) {
  RecursiveFilter::Configuration config;

  auto busy_input_op = std::make_shared<MockBayesianOperation>("input/busy");
  busy_input_op->SetExecResult(Err(Error::kBusy));  // Always return BUSY
  auto input_op = std::make_shared<MockBayesianOperation>("input");
  auto preprocess_op = std::make_shared<MockBayesianOperation>("preprocess");
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");

  config.input_stages.push_back(busy_input_op);
  config.input_stages.push_back(input_op);
  config.preprocess_stages.push_back(preprocess_op);
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(100);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);

  // A busy input stage does not suppress preprocessing on its own.
  EXPECT_GE(preprocess_op->ExecCount(), 1);
  EXPECT_GE(solver_op->ExecCount(), 1);
}

// Test that a filter without input stages is rejected
TEST_F(RecursiveFilterTest, RunWithoutInputStagesFails) {
  RecursiveFilter::Configuration config;
  auto solver_op = std::make_shared<MockBayesianOperation>("solver");
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(10);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);

  ASSERT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kFailure);
  EXPECT_THAT(status.error().GetDescription(), testing::HasSubstr("input stage"));
  // Nothing is executed when the configuration is rejected.
  EXPECT_EQ(solver_op->ExecCount(), 0);
  EXPECT_FALSE(solver_op->InitCalled());
}

// Test that a filter without a solver stage is rejected
TEST_F(RecursiveFilterTest, RunWithoutSolverStageFails) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(10);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);

  ASSERT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kFailure);
  EXPECT_THAT(status.error().GetDescription(), testing::HasSubstr("solver stage"));
}

// Test BUSY status handling on preprocess stages
TEST_F(RecursiveFilterTest, PreprocessStageBusyHandling) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);

  auto preprocess_op = std::make_shared<MockBayesianOperation>("preprocess");
  preprocess_op->SetExecResult(Err(Error::kBusy));

  auto solver_op = std::make_shared<MockBayesianOperation>("solver");

  config.preprocess_stages.push_back(preprocess_op);
  config.solver_stage = solver_op;
  config.rate = 10.0;

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(100);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);

  // BUSY in preprocess skips the solver on every iteration but the first, which
  // always solves to bootstrap from the initialized state.
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);
  EXPECT_GE(preprocess_op->ExecCount(), 1);
  EXPECT_EQ(solver_op->ExecCount(), 1);
}

// Test rate is created with correct frequency
TEST_F(RecursiveFilterTest, RateCreatedWithCorrectFrequency) {
  RecursiveFilter::Configuration config;
  AddInputStage(config);

  auto solver_op = std::make_shared<MockBayesianOperation>("solver");
  config.solver_stage = solver_op;
  config.rate = 50.0;  // Distinctive rate

  double captured_rate = 0.0;
  ON_CALL(*middleware_, CreateRate(_)).WillByDefault([this, &captured_rate](double freq) {
    captured_rate = freq;
    return std::make_unique<MockRate>(freq, clock_.get());
  });

  RecursiveFilter filter(middleware_, std::move(config));

  auto deadline = clock_->Now() + std::chrono::milliseconds(50);
  ComputeBudget budget(deadline, {});

  Result status = filter.Run(budget);
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);
  EXPECT_DOUBLE_EQ(captured_rate, 50.0);
}

}  // namespace

// Registerable operation for LoadFromParams testing
class RegisterableOperation : public beluga_estimation::BayesianOperation_<double> {
 public:
  explicit RegisterableOperation(
      std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware,
      std::string_view prefix)
      : middleware_(std::move(middleware)), prefix_(prefix) {}

  beluga_estimation::Result Initialize(const beluga_estimation::ComputeBudget&) override {
    return beluga_estimation::Ok();
  }
  beluga_estimation::Result Execute(
      beluga_estimation::Context_<double>& /*context*/,
      const beluga_estimation::ComputeBudget&) override {
    return beluga_estimation::Ok();
  }
  beluga_estimation::Result Finalize(const beluga_estimation::ComputeBudget&) override {
    return beluga_estimation::Ok();
  }

  static beluga_estimation::Result_<std::unique_ptr<RegisterableOperation>> Make(
      std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<RegisterableOperation>(std::move(middleware), prefix);
  }

  std::string GetPrefix() const { return prefix_; }

 private:
  std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware_;
  std::string prefix_;
};

// No need for BELUGA_ESTIMATION_EXPORT_TYPE - BayesianOperation is already exported in operation.hpp
BELUGA_ESTIMATION_REGISTER_TYPE(RegisterableOperation, beluga_estimation::BayesianOperation_<double>);

namespace {

// Test LoadFromParams constructor path
TEST_F(RecursiveFilterTest, LoadFromParamsConstructor) {
  // Set up parameter expectations - using the ParameterType overload
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("filter.init.stages", ParameterType::kStringArray))
      .WillOnce(::testing::Return(ParameterValue{std::vector<std::string>{"init_stage"}}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("filter.init.init_stage.type", ParameterType::kString))
      .WillOnce(::testing::Return(ParameterValue{std::string("RegisterableOperation")}));

  EXPECT_CALL(*middleware_->parameters, DeclareParameter("filter.input.stages", ParameterType::kStringArray))
      .WillOnce(::testing::Return(ParameterValue{std::vector<std::string>{"input_stage"}}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("filter.input.input_stage.type", ParameterType::kString))
      .WillOnce(::testing::Return(ParameterValue{std::string("RegisterableOperation")}));

  EXPECT_CALL(
      *middleware_->parameters,
      DeclareParameter("filter.preprocess.stages", ParameterValue{std::vector<std::string>{}}))
      .WillOnce(::testing::Return(ParameterValue{std::vector<std::string>{"preprocess_stage"}}));
  EXPECT_CALL(
      *middleware_->parameters, DeclareParameter("filter.preprocess.preprocess_stage.type", ParameterType::kString))
      .WillOnce(::testing::Return(ParameterValue{std::string("RegisterableOperation")}));

  EXPECT_CALL(*middleware_->parameters, DeclareParameter("filter.solver.type", ParameterType::kString))
      .WillOnce(::testing::Return(ParameterValue{std::string("RegisterableOperation")}));

  EXPECT_CALL(*middleware_->parameters, DeclareParameter("filter.output.stages", ParameterType::kStringArray))
      .WillOnce(::testing::Return(ParameterValue{std::vector<std::string>{"output_stage"}}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("filter.output.output_stage.type", ParameterType::kString))
      .WillOnce(::testing::Return(ParameterValue{std::string("RegisterableOperation")}));

  EXPECT_CALL(*middleware_->parameters, DeclareParameter("filter.rate", ParameterType::kDouble))
      .WillOnce(::testing::Return(ParameterValue{10.0}));

  // Construct using the Make path
  auto filter_result = RecursiveFilter::Make(middleware_, "filter.");
  ASSERT_TRUE(filter_result);
  auto filter = Unwrap(filter_result);

  // Run briefly to verify it was constructed correctly
  auto deadline = clock_->Now() + std::chrono::milliseconds(1);
  ComputeBudget budget(deadline, {});
  Result status = filter->Run(budget);

  // Should run and timeout (not crash)
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kTimeout);
}

}  // namespace
