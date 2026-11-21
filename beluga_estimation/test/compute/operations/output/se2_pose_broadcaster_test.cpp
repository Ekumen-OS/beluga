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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <sophus/se2.hpp>

#include <beluga_estimation/compute/context.hpp>
#include <beluga_estimation/compute/operations/output/se2_pose_broadcaster.hpp>
#include <beluga_estimation/core/estimate_store.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/mocks/mock_clock.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace {

using namespace beluga_estimation;
using ::testing::_;
using ::testing::Return;

// Test fixture
class SE2PoseBroadcasterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    middleware_ = std::make_shared<MockMiddleware>();
    clock_ = std::make_shared<MockClock>();

    ON_CALL(*middleware_, GetClockInterface()).WillByDefault(Return(clock_.get()));
    EXPECT_CALL(*middleware_, GetClockInterface()).Times(::testing::AnyNumber());

    ON_CALL(*middleware_->transform, BroadcastTransform(_, _, _, _)).WillByDefault(Return());
    EXPECT_CALL(*middleware_->transform, BroadcastTransform(_, _, _, _)).Times(::testing::AnyNumber());

    ON_CALL(*middleware_->logger, Log(_, _)).WillByDefault(Return());
    EXPECT_CALL(*middleware_->logger, Log(_, _)).Times(::testing::AnyNumber());

    config_.variable_name = "robot_pose";
    config_.max_broadcast_rate = 10.0;
    config_.target_frame = "map";
    config_.source_frame = "base_link";
  }

  std::shared_ptr<MockMiddleware> middleware_;
  std::shared_ptr<MockClock> clock_;
  SE2PoseBroadcaster::Configuration config_;
};

// Test construction
TEST_F(SE2PoseBroadcasterTest, Construction) {
  EXPECT_NO_THROW({ SE2PoseBroadcaster broadcaster(middleware_, config_); });
}

// Test construction from parameters
TEST_F(SE2PoseBroadcasterTest, ConstructionFromParams) {
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.variable_name", ParameterType::kString))
      .WillOnce(Return(ParameterValue{std::string("robot_pose")}));
  EXPECT_CALL(
      *middleware_->parameters, DeclareParameter("test.max_broadcast_rate", testing::An<const ParameterValue&>()))
      .WillOnce(Return(ParameterValue{10.0}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.target_frame", ParameterType::kString))
      .WillOnce(Return(ParameterValue{std::string("map")}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.source_frame", ParameterType::kString))
      .WillOnce(Return(ParameterValue{std::string("base_link")}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.target_estimate", testing::An<const ParameterValue&>()))
      .WillOnce(Return(ParameterValue{std::string("")}));

  EXPECT_NO_THROW({ SE2PoseBroadcaster broadcaster(middleware_, "test."); });
}

// Test Initialize / Finalize
TEST_F(SE2PoseBroadcasterTest, Initialize) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  ComputeBudget budget = ComputeBudget::Unlimited();
  EXPECT_TRUE(broadcaster.Initialize(budget));
}

// Test execution on empty DBN
TEST_F(SE2PoseBroadcasterTest, ExecuteOnEmptyDBN) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);

  Context ctx;
  ComputeBudget budget = ComputeBudget::Unlimited();

  // Initialize first
  ASSERT_TRUE(broadcaster.Initialize(budget));

  // Execute on empty network - should return ERROR (variable not found)
  Result status = broadcaster.Execute(ctx, budget);
  EXPECT_EQ(status.error(), Error::kFailure);
}

// Test execution with SE2Variable in DBN but without an estimate
TEST_F(SE2PoseBroadcasterTest, ExecuteWithVariable) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);

  Context ctx;
  ComputeBudget budget = ComputeBudget::Unlimited();

  // Create and add an SE2Variable
  Identifier var_id{"robot_pose", std::chrono::system_clock::now()};
  auto variable = std::make_shared<SE2Variable>(var_id);

  ctx.dbn.Add(variable);

  // Initialize and execute
  ASSERT_TRUE(broadcaster.Initialize(budget));

  // Without an estimate, execution should return ERROR
  Result status = broadcaster.Execute(ctx, budget);
  EXPECT_EQ(status.error(), Error::kFailure);
}

// Test finalization lifecycle
TEST_F(SE2PoseBroadcasterTest, Finalize) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  ComputeBudget budget = ComputeBudget::Unlimited();
  Result status = broadcaster.Finalize(budget);

  EXPECT_TRUE(status);
}

// Test lifecycle: Initialize -> Execute (variable, no estimate) -> Finalize
TEST_F(SE2PoseBroadcasterTest, LifecycleWithoutEstimate) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);

  Context ctx;
  ComputeBudget budget = ComputeBudget::Unlimited();

  // Add a variable to the DBN
  Identifier var_id{"robot_pose", std::chrono::system_clock::now()};
  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  EXPECT_TRUE(broadcaster.Initialize(budget));
  EXPECT_EQ(broadcaster.Execute(ctx, budget).error(), Error::kFailure);
  EXPECT_TRUE(broadcaster.Finalize(budget));
}

// Error: variable not in DBN
TEST_F(SE2PoseBroadcasterTest, ExecuteOnEmptyDBNReturnsError) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  Context ctx;
  ComputeBudget budget = ComputeBudget::Unlimited();

  ASSERT_TRUE(broadcaster.Initialize(budget));
  EXPECT_EQ(broadcaster.Execute(ctx, budget).error(), Error::kFailure);
}

// Error: variable in DBN, no matching estimate
TEST_F(SE2PoseBroadcasterTest, ExecuteWithNoEstimateReturnsError) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  Context ctx;
  ComputeBudget budget = ComputeBudget::Unlimited();

  Identifier var_id{"robot_pose", TimePoint{std::chrono::seconds(1)}};
  ctx.dbn.Add(std::make_shared<SE2Variable>(var_id));

  ASSERT_TRUE(broadcaster.Initialize(budget));
  EXPECT_EQ(broadcaster.Execute(ctx, budget).error(), Error::kFailure);
}

// Success: variable found, auto-detected estimate, broadcast called
TEST_F(SE2PoseBroadcasterTest, ExecuteSuccessAutoEstimate) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  Context ctx;
  Identifier var_id{"robot_pose", TimePoint{std::chrono::seconds(1)}};

  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  Vector3d mean = Vector3d::Zero();
  Matrix3d cov = Matrix3d::Identity();
  auto tangent_dist = std::make_shared<NormalDistribution_<double, 3>>(mean, cov);

  auto tangent = std::make_shared<SE2TangentDistribution>(tangent_dist);

  auto estimate = std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{variable}, tangent);
  ctx.estimates.Add(var_id, std::move(estimate));

  ComputeBudget budget = ComputeBudget::Unlimited();

  EXPECT_CALL(*middleware_->transform, BroadcastTransform(config_.target_frame, config_.source_frame, _, _)).Times(1);

  ASSERT_TRUE(broadcaster.Initialize(budget));
  EXPECT_TRUE(broadcaster.Execute(ctx, budget));
}

// Success: explicit target_estimate name
TEST_F(SE2PoseBroadcasterTest, ExecuteSuccessWithExplicitTargetEstimate) {
  config_.target_estimate = "filtered_pose";
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  Context ctx;
  Identifier var_id{"robot_pose", TimePoint{std::chrono::seconds(1)}};

  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  Vector3d mean = Vector3d::Zero();
  Matrix3d cov = Matrix3d::Identity();
  auto tangent_dist = std::make_shared<NormalDistribution_<double, 3>>(mean, cov);
  auto tangent = std::make_shared<SE2TangentDistribution>(tangent_dist);
  auto estimate = std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{variable}, tangent);
  ctx.estimates.Add(Identifier{"filtered_pose", TimePoint{std::chrono::seconds(2)}}, std::move(estimate));

  ComputeBudget budget = ComputeBudget::Unlimited();

  EXPECT_CALL(*middleware_->transform, BroadcastTransform(config_.target_frame, config_.source_frame, _, _)).Times(1);

  ASSERT_TRUE(broadcaster.Initialize(budget));
  EXPECT_TRUE(broadcaster.Execute(ctx, budget));
}

// Error: explicit target_estimate name not found
TEST_F(SE2PoseBroadcasterTest, ExecuteWithMissingTargetEstimateReturnsError) {
  config_.target_estimate = "missing_estimate";
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  Context ctx;
  Identifier var_id{"robot_pose", TimePoint{std::chrono::seconds(1)}};

  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  Vector3d mean = Vector3d::Zero();
  Matrix3d cov = Matrix3d::Identity();
  auto tangent_dist = std::make_shared<NormalDistribution_<double, 3>>(mean, cov);

  auto tangent = std::make_shared<SE2TangentDistribution>(tangent_dist);

  auto estimate = std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{variable}, tangent);
  ctx.estimates.Add(var_id, std::move(estimate));

  ComputeBudget budget = ComputeBudget::Unlimited();

  ASSERT_TRUE(broadcaster.Initialize(budget));
  EXPECT_EQ(broadcaster.Execute(ctx, budget).error(), Error::kFailure);
}

// Throttling: second immediate execution skips broadcast
TEST_F(SE2PoseBroadcasterTest, BroadcastRateThrottling) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  Context ctx;
  Identifier var_id{"robot_pose", TimePoint{std::chrono::seconds(1)}};

  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  Vector3d mean = Vector3d::Zero();
  Matrix3d cov = Matrix3d::Identity();
  auto tangent_dist = std::make_shared<NormalDistribution_<double, 3>>(mean, cov);

  auto tangent = std::make_shared<SE2TangentDistribution>(tangent_dist);

  auto estimate = std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{variable}, tangent);
  ctx.estimates.Add(var_id, std::move(estimate));

  ComputeBudget budget = ComputeBudget::Unlimited();

  EXPECT_CALL(*middleware_->transform, BroadcastTransform(config_.target_frame, config_.source_frame, _, _)).Times(1);

  ASSERT_TRUE(broadcaster.Initialize(budget));
  EXPECT_TRUE(broadcaster.Execute(ctx, budget));
  EXPECT_TRUE(broadcaster.Execute(ctx, budget));
}

// Full lifecycle
TEST_F(SE2PoseBroadcasterTest, FullLifecycle) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  Context ctx;
  Identifier var_id{"robot_pose", TimePoint{std::chrono::seconds(1)}};

  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  Vector3d mean = Vector3d::Zero();
  Matrix3d cov = Matrix3d::Identity();
  auto tangent_dist = std::make_shared<NormalDistribution_<double, 3>>(mean, cov);

  auto tangent = std::make_shared<SE2TangentDistribution>(tangent_dist);

  auto estimate = std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{variable}, tangent);
  ctx.estimates.Add(var_id, std::move(estimate));

  ComputeBudget budget = ComputeBudget::Unlimited();

  EXPECT_CALL(*middleware_->transform, BroadcastTransform(config_.target_frame, config_.source_frame, _, _)).Times(1);

  EXPECT_TRUE(broadcaster.Initialize(budget));
  EXPECT_TRUE(broadcaster.Execute(ctx, budget));
  EXPECT_TRUE(broadcaster.Finalize(budget));
}

// Broadcast rate enforcement at 10Hz
TEST(SE2PoseBroadcasterRateTest, BroadcastRateEnforcement) {
  auto test_middleware = std::make_shared<MockMiddleware>();
  auto test_clock = std::make_shared<MockClock>();

  ON_CALL(*test_middleware, GetClockInterface()).WillByDefault(Return(test_clock.get()));
  EXPECT_CALL(*test_middleware, GetClockInterface()).Times(::testing::AtLeast(1));

  EXPECT_CALL(*test_middleware->transform, BroadcastTransform(_, _, _, _)).Times(10);

  SE2PoseBroadcaster::Configuration config;
  config.variable_name = "robot_pose";
  config.max_broadcast_rate = 10.0;
  config.target_frame = "map";
  config.source_frame = "base_link";

  SE2PoseBroadcaster broadcaster(test_middleware, config);
  ComputeBudget budget = ComputeBudget::Unlimited();

  Context ctx;
  Identifier var_id{"robot_pose", std::chrono::system_clock::now()};
  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  // Add a matching estimate so Execute succeeds.
  Vector3d mean = Vector3d::Zero();
  Matrix3d cov = Matrix3d::Identity();
  auto tangent_dist = std::make_shared<NormalDistribution_<double, 3>>(mean, cov);
  auto tangent = std::make_shared<SE2TangentDistribution>(tangent_dist);
  auto estimate = std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{variable}, tangent);
  ctx.estimates.Add(var_id, std::move(estimate));

  ASSERT_TRUE(broadcaster.Initialize(budget));

  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(broadcaster.Execute(ctx, budget));
    test_clock->Advance(std::chrono::milliseconds(10));
  }
}

// Zero rate broadcasts always
TEST(SE2PoseBroadcasterRateTest, ZeroBroadcastRateBroadcastsAlways) {
  auto test_middleware = std::make_shared<MockMiddleware>();
  auto test_clock = std::make_shared<MockClock>();

  ON_CALL(*test_middleware, GetClockInterface()).WillByDefault(Return(test_clock.get()));
  EXPECT_CALL(*test_middleware, GetClockInterface()).Times(::testing::AtLeast(1));

  const int num_executions = 50;
  EXPECT_CALL(*test_middleware->transform, BroadcastTransform(_, _, _, _)).Times(num_executions);

  SE2PoseBroadcaster::Configuration config;
  config.variable_name = "robot_pose";
  config.max_broadcast_rate = 0.0;
  config.target_frame = "map";
  config.source_frame = "base_link";

  SE2PoseBroadcaster broadcaster(test_middleware, config);
  ComputeBudget budget = ComputeBudget::Unlimited();

  Context ctx;
  Identifier var_id{"robot_pose", std::chrono::system_clock::now()};
  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  // Add a matching estimate so Execute succeeds.
  Vector3d mean = Vector3d::Zero();
  Matrix3d cov = Matrix3d::Identity();
  auto tangent_dist = std::make_shared<NormalDistribution_<double, 3>>(mean, cov);
  auto tangent = std::make_shared<SE2TangentDistribution>(tangent_dist);
  auto estimate = std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{variable}, tangent);
  ctx.estimates.Add(var_id, std::move(estimate));

  ASSERT_TRUE(broadcaster.Initialize(budget));

  for (int i = 0; i < num_executions; ++i) {
    EXPECT_TRUE(broadcaster.Execute(ctx, budget));
    test_clock->Advance(std::chrono::milliseconds(10));
  }
}

// Success: broadcast is reported at debug level
TEST_F(SE2PoseBroadcasterTest, EmitsDebugMessageWhenBroadcasting) {
  SE2PoseBroadcaster broadcaster(middleware_, config_);
  Context ctx;
  Identifier var_id{"robot_pose", TimePoint{std::chrono::seconds(1)}};

  auto variable = std::make_shared<SE2Variable>(var_id);
  ctx.dbn.Add(variable);

  auto tangent_dist = std::make_shared<NormalDistribution_<double, 3>>(Vector3d::Zero(), Matrix3d::Identity());
  auto tangent = std::make_shared<SE2TangentDistribution>(tangent_dist);
  auto estimate = std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{variable}, tangent);
  ctx.estimates.Add(var_id, std::move(estimate));

  std::vector<std::string> debug_messages;
  ON_CALL(*middleware_->logger, Log(LogLevel::kDebug, _))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });

  ComputeBudget budget = ComputeBudget::Unlimited();

  ASSERT_TRUE(broadcaster.Initialize(budget));
  ASSERT_TRUE(broadcaster.Execute(ctx, budget));

  EXPECT_THAT(
      debug_messages,
      ::testing::Contains("SE2PoseBroadcaster: broadcast 'base_link' -> 'map' for variable 'robot_pose'"));
}

}  // namespace
