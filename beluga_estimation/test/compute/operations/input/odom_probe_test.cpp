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
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/compute/context.hpp>
#include <beluga_estimation/compute/operations/input/odom_probe.hpp>
#include <beluga_estimation/constraints/normal_likelihood_constraint.hpp>
#include <beluga_estimation/constraints/se2_normal_pose_likelihood_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/variable.hpp>
#include <beluga_estimation/mocks/mock_clock.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/mocks/mock_odometry.hpp>
#include <beluga_estimation/mocks/mock_subscriber.hpp>
#include <beluga_estimation/msg/odometry.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

using namespace beluga_estimation;
using ::testing::_;
using ::testing::Eq;
using ::testing::Return;

class OdomProbeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    middleware_ = std::make_shared<MockMiddleware>();
    clock_ = std::make_shared<MockClock>();

    ON_CALL(*middleware_, GetClockInterface()).WillByDefault(Return(clock_.get()));
    EXPECT_CALL(*middleware_, GetClockInterface()).Times(::testing::AnyNumber());
  }

  // A configuration that processes the pose channel only.
  static OdomProbe::Configuration MakeConfig() {
    OdomProbe::Configuration config;
    config.device_name = "odom";
    config.topic_name = "odom";
    config.use_pose = true;
    config.twist_mask = {false, false, false};
    return config;
  }

  // Stubs the parameter declarations LoadFromParams reads, with the given
  // transform_timeout value (in seconds).
  void ExpectDefaultParams(double transform_timeout) {
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.device_name", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("odom")}));
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.topic_name", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("odom")}));
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.world_frame", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("")}));
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.base_frame", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("")}));
    EXPECT_CALL(
        *middleware_->parameters, DeclareParameter("test.transform_timeout", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{transform_timeout}));
    EXPECT_CALL(
        *middleware_->parameters, DeclareParameter("test.pose_variable_name", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("pose")}));
    EXPECT_CALL(
        *middleware_->parameters, DeclareParameter("test.velocity_variable_name", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("velocity")}));
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.use_pose", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{true}));
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.twist_mask", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::vector<bool>{true, true, true}}));
  }

  // Sets up the mock subscriber and captures the callback handed to it.
  void SetUpSubscriber() {
    EXPECT_CALL(*middleware_->topics, CreateSubscriberImpl(_, _, _))
        .WillOnce(testing::DoAll(
            testing::SaveArg<2>(&captured_callback_),
            testing::Return(std::make_shared<MockSubscriber<beluga_estimation::msg::Odometry>>())));
  }

  // Injects a pose-only test message through the captured subscriber callback.
  void InjectPoseMessage(beluga_estimation::TimePoint stamp, const beluga_estimation::SE2d& pose = {}) {
    captured_callback_(std::make_shared<const MockOdometry>(stamp, pose));
  }

  std::shared_ptr<MockMiddleware> middleware_;
  std::shared_ptr<MockClock> clock_;
  beluga_estimation::SubscriberCallback<beluga_estimation::Message> captured_callback_;
  ComputeBudget budget_{ComputeBudget::Unlimited()};
};

// =============================================================================
// Basic Compilation and Type Tests
// =============================================================================

TEST_F(OdomProbeTest, CanInstantiateConfigurationStruct) {
  OdomProbe::Configuration config;
  config.device_name = "test_odom";
  config.topic_name = "test/odom";
  config.world_frame = "";
  config.use_pose = true;
  config.twist_mask = {true, true, true};

  EXPECT_EQ(config.device_name, "test_odom");
  EXPECT_EQ(config.topic_name, "test/odom");
  EXPECT_EQ(config.world_frame, "");
  EXPECT_EQ(config.transform_timeout.count(), 0);
  EXPECT_TRUE(config.use_pose);
  EXPECT_EQ(config.twist_mask, std::vector<bool>({true, true, true}));
}

TEST_F(OdomProbeTest, ConstructionWithExplicitConfig) {
  EXPECT_NO_THROW({ OdomProbe probe(middleware_, MakeConfig()); });
}

TEST_F(OdomProbeTest, ConstructionFromParams) {
  ExpectDefaultParams(0.0);
  EXPECT_NO_THROW({ OdomProbe probe(middleware_, "test."); });
}

// =============================================================================
// Lifecycle Tests
// =============================================================================

TEST_F(OdomProbeTest, InitializeCreatesSubscription) {
  OdomProbe probe(middleware_, MakeConfig());

  // Initialize creates a subscriber for the configured topic.
  EXPECT_CALL(*middleware_->topics, CreateSubscriberImpl(_, _, _))
      .WillOnce(Return(std::make_shared<MockSubscriber<beluga_estimation::msg::Odometry>>()));

  EXPECT_TRUE(probe.Initialize(budget_));
}

TEST_F(OdomProbeTest, FinalizeDestroysSubscription) {
  OdomProbe probe(middleware_, MakeConfig());

  EXPECT_CALL(*middleware_->topics, CreateSubscriberImpl(_, _, _))
      .WillOnce(Return(std::make_shared<MockSubscriber<beluga_estimation::msg::Odometry>>()));

  ASSERT_TRUE(probe.Initialize(budget_));
  EXPECT_TRUE(probe.Finalize(budget_));
}

TEST_F(OdomProbeTest, ExecuteWithEmptyQueueReturnsBusy) {
  OdomProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;

  // With an empty message queue, Execute should return BUSY.
  EXPECT_EQ(probe.Execute(ctx, budget_).error(), Error::kBusy);
}

// =============================================================================
// Message Processing Tests
// =============================================================================

TEST_F(OdomProbeTest, ProcessPoseMessage) {
  OdomProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  // Create and inject a test odometry message.
  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(
      timestamp, beluga_estimation::SE2d(beluga_estimation::SO2d(0.5), beluga_estimation::Vector2d(1.0, 2.0)));

  // Execute should process the message.
  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // Verify the SE(2) variable and pose likelihood constraint were added.
  Identifier variable_id{std::string(OdomProbe::kPoseName), timestamp};
  Identifier constraint_id{"odom/pose/measurement", timestamp};
  auto variable_result = ctx.dbn.GetVariable<SE2Variable_<double>>(variable_id);
  ASSERT_TRUE(variable_result);
  EXPECT_NE(Unwrap(variable_result), nullptr);
  auto pose_constraint_result = ctx.dbn.GetConstraint<SE2NormalPoseLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(pose_constraint_result);
  auto pose_constraint = Unwrap(pose_constraint_result);
  EXPECT_NE(pose_constraint, nullptr);
}

TEST_F(OdomProbeTest, ProcessVelocityMessage) {
  auto config = MakeConfig();
  config.use_pose = false;
  config.twist_mask = {true, true, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  const beluga_estimation::Vector3d linear_velocity(0.5, -0.25, 0.0);
  const beluga_estimation::Vector3d angular_velocity(0.0, 0.0, 0.5);
  captured_callback_(std::make_shared<const MockOdometry>(
      timestamp, beluga_estimation::SE2d{}, beluga_estimation::Matrix3d::Identity() * 0.01,
      beluga_estimation::Matrix3d::Identity() * 0.02, linear_velocity, angular_velocity));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // The twist is measured on the shared 3D velocity variable.
  Identifier variable_id{std::string(OdomProbe::kVelocityName), timestamp};
  Identifier constraint_id{"odom/velocity/measurement", timestamp};
  auto velocity_variable_result = ctx.dbn.GetVariable<EuclideanVariable_<double>>(variable_id);
  ASSERT_TRUE(velocity_variable_result);
  auto velocity_variable = Unwrap(velocity_variable_result);
  ASSERT_NE(velocity_variable, nullptr);
  EXPECT_EQ(velocity_variable->Size(), 3u);
  auto velocity_constraint_result = ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(velocity_constraint_result);
  auto velocity_constraint = Unwrap(velocity_constraint_result);
  ASSERT_NE(velocity_constraint, nullptr);
  EXPECT_EQ(velocity_constraint->ResidualSize(), 3u);

  // The measurement keeps the planar components and the full covariance block.
  beluga_estimation::VectorXd residual(3);
  std::vector<Vector1d> empty;
  std::vector<Vector3d> state = {{0.5, -0.25, 0.5}};
  velocity_constraint->CalcResidual(to_view(empty), to_view(state), to_view(residual));
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));

  beluga_estimation::MatrixXd covariance(3, 3);
  velocity_constraint->CalcCovariance(covariance);
  EXPECT_NEAR(covariance(0, 0), 0.01, 1e-12);
  EXPECT_NEAR(covariance(1, 1), 0.01, 1e-12);
  EXPECT_NEAR(covariance(2, 2), 0.02, 1e-12);
}

TEST_F(OdomProbeTest, ProcessLinearVelocityMaskOnly) {
  auto config = MakeConfig();
  config.use_pose = false;
  config.twist_mask = {true, true, false};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  captured_callback_(std::make_shared<const MockOdometry>(
      timestamp, beluga_estimation::SE2d{}, beluga_estimation::Matrix3d::Identity() * 0.01,
      beluga_estimation::Matrix3d::Identity() * 0.01, beluga_estimation::Vector3d(0.5, -0.25, 0.0),
      beluga_estimation::Vector3d(0.0, 0.0, 0.5)));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier constraint_id{"odom/velocity/measurement", timestamp};
  auto constraint = Unwrap(ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id));
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->ResidualSize(), 2u);

  // Only the linear coordinates are observed; the yaw rate is ignored.
  std::vector<Vector1d> empty;
  std::vector<Vector3d> state = {{0.5, -0.25, 999.0}};
  beluga_estimation::VectorXd residual(2);
  constraint->CalcResidual(to_view(empty), to_view(state), to_view(residual));
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));
}

TEST_F(OdomProbeTest, ProcessAngularVelocityMaskOnly) {
  auto config = MakeConfig();
  config.use_pose = false;
  config.twist_mask = {false, false, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  captured_callback_(std::make_shared<const MockOdometry>(
      timestamp, beluga_estimation::SE2d{}, beluga_estimation::Matrix3d::Identity() * 0.01,
      beluga_estimation::Matrix3d::Identity() * 0.01, beluga_estimation::Vector3d(0.5, -0.25, 0.0),
      beluga_estimation::Vector3d(0.0, 0.0, 0.5)));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier constraint_id{"odom/velocity/measurement", timestamp};
  auto constraint = Unwrap(ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id));
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->ResidualSize(), 1u);

  // Only the yaw rate is observed; the linear coordinates are ignored.
  std::vector<Vector1d> empty;
  std::vector<Vector3d> state = {{999.0, 999.0, 0.5}};
  beluga_estimation::VectorXd residual(1);
  constraint->CalcResidual(to_view(empty), to_view(state), to_view(residual));
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));
}

TEST_F(OdomProbeTest, ProcessNonContiguousVelocityMask) {
  auto config = MakeConfig();
  config.use_pose = false;
  config.twist_mask = {true, false, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  captured_callback_(std::make_shared<const MockOdometry>(
      timestamp, beluga_estimation::SE2d{}, beluga_estimation::Matrix3d::Identity() * 0.01,
      beluga_estimation::Matrix3d::Identity() * 0.01, beluga_estimation::Vector3d(0.5, -0.25, 0.0),
      beluga_estimation::Vector3d(0.0, 0.0, 0.5)));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier constraint_id{"odom/velocity/measurement", timestamp};
  auto constraint = Unwrap(ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id));
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->ResidualSize(), 2u);

  // vx and omega are observed; vy is skipped even though it lies between them.
  std::vector<Vector1d> empty;
  std::vector<Vector3d> state = {{0.5, 999.0, 0.5}};
  beluga_estimation::VectorXd residual(2);
  constraint->CalcResidual(to_view(empty), to_view(state), to_view(residual));
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));
}

TEST_F(OdomProbeTest, UsesConfiguredVariableNames) {
  auto config = MakeConfig();
  config.use_pose = true;
  config.twist_mask = {true, true, true};
  config.pose_variable_name = "body_pose";
  config.velocity_variable_name = "body_twist";
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  ASSERT_NE((ctx.dbn.GetVariable<SE2Variable_<double>>(Identifier{"body_pose", timestamp})), nullptr);
  {
    auto twist_result = ctx.dbn.GetVariable<EuclideanVariable_<double>>(Identifier{"body_twist", timestamp});
    ASSERT_TRUE(twist_result);
    auto twist = Unwrap(twist_result);
    ASSERT_NE(twist, nullptr);
    EXPECT_EQ(twist->Size(), 3u);
  }
  ASSERT_EQ(ctx.dbn.GetVariable(Identifier{std::string(OdomProbe::kPoseName), timestamp}), nullptr);
  ASSERT_EQ(ctx.dbn.GetVariable(Identifier{std::string(OdomProbe::kVelocityName), timestamp}), nullptr);

  // Constraint names derive from the configured variable names, not the defaults.
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"odom/body_pose/measurement", timestamp}), nullptr);
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"odom/body_twist/measurement", timestamp}), nullptr);
  EXPECT_EQ(ctx.dbn.GetConstraint(Identifier{"odom/pose/measurement", timestamp}), nullptr);
  EXPECT_EQ(ctx.dbn.GetConstraint(Identifier{"odom/velocity/measurement", timestamp}), nullptr);
}

TEST_F(OdomProbeTest, MultiAgentProbesDoNotCollide) {
  auto agent0_config = MakeConfig();
  agent0_config.use_pose = true;
  agent0_config.twist_mask = {true, true, true};
  agent0_config.pose_variable_name = "agent0/pose";
  agent0_config.velocity_variable_name = "agent0/velocity";

  auto agent1_config = agent0_config;
  agent1_config.pose_variable_name = "agent1/pose";
  agent1_config.velocity_variable_name = "agent1/velocity";

  // Both agents keep the default device name; only the variable names differ.
  OdomProbe agent0(middleware_, agent0_config);
  OdomProbe agent1(middleware_, agent1_config);

  Context_<double> ctx;
  const auto timestamp = TimePoint{std::chrono::seconds(100)};

  SetUpSubscriber();
  ASSERT_TRUE(agent0.Initialize(budget_));
  InjectPoseMessage(timestamp);
  ASSERT_TRUE(agent0.Execute(ctx, budget_));

  SetUpSubscriber();
  ASSERT_TRUE(agent1.Initialize(budget_));
  InjectPoseMessage(timestamp);
  ASSERT_TRUE(agent1.Execute(ctx, budget_));

  // A name clash would have made the second agent's insertions fail.
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"odom/agent0/pose/measurement", timestamp}), nullptr);
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"odom/agent0/velocity/measurement", timestamp}), nullptr);
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"odom/agent1/pose/measurement", timestamp}), nullptr);
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"odom/agent1/velocity/measurement", timestamp}), nullptr);
}

TEST_F(OdomProbeTest, ProcessAllElements) {
  auto config = MakeConfig();
  config.twist_mask = {true, true, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // Verify the pose and the shared velocity variable were added to the DBN.
  Identifier pose_id{std::string(OdomProbe::kPoseName), timestamp};
  EXPECT_NE(ctx.dbn.GetVariable(pose_id), nullptr);
  Identifier velocity_id{std::string(OdomProbe::kVelocityName), timestamp};
  auto velocity = ctx.dbn.GetVariable<EuclideanVariable_<double>>(velocity_id);
  ASSERT_TRUE(velocity);
  auto velocity_ptr = Unwrap(velocity);
  EXPECT_NE(velocity_ptr, nullptr);
  EXPECT_EQ(velocity_ptr->Size(), 3u);
}

TEST_F(OdomProbeTest, EmitsDebugSummaryAfterProcessingMessages) {
  OdomProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;
  SetUpSubscriber();

  std::vector<std::string> debug_messages;
  ON_CALL(*middleware_->logger, Log(LogLevel::kDebug, _))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });
  EXPECT_CALL(*middleware_->logger, Log(LogLevel::kDebug, _)).Times(::testing::AnyNumber());

  ASSERT_TRUE(probe.Initialize(budget_));

  InjectPoseMessage(TimePoint{std::chrono::seconds(100)});

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  EXPECT_THAT(debug_messages, ::testing::Contains("OdomProbe: processed 1 message(s) from 'odom' device"));
}

// =============================================================================
// Transform Lookup Tests
// =============================================================================

TEST_F(OdomProbeTest, PoseTransformIsApplied) {
  auto config = MakeConfig();
  config.world_frame = "map";
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // The pose lives in the header frame ("odom"); expect the transform from
  // "odom" to the reference frame "map": a 90° rotation about z and a
  // translation of 1 m along x.
  const beluga_estimation::SE3d transform(
      beluga_estimation::SO3d(
          beluga_estimation::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, beluga_estimation::Vector3d::UnitZ()))),
      beluga_estimation::Vector3d(1.0, 0.0, 0.0));
  EXPECT_CALL(*middleware_->transform, LookupTransform(Eq("map"), Eq("odom"), _, _)).WillOnce(Return(transform));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);  // Pose at the origin.

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // The measurement must be the transformed pose: T * identity = (1, 0, π/2).
  Identifier constraint_id{"odom/pose/measurement", timestamp};
  auto constraint_result = ctx.dbn.GetConstraint<SE2NormalPoseLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint_result);
  auto constraint = Unwrap(constraint_result);
  EXPECT_NE(constraint, nullptr);

  const beluga_estimation::SE2d expected_mean(
      beluga_estimation::SO2d(M_PI / 2.0), beluga_estimation::Vector2d(1.0, 0.0));
  beluga_estimation::Vector3d residual;
  constraint->CalcResidual(expected_mean, residual);
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));

  // The untransformed pose is not consistent with the measurement anymore.
  beluga_estimation::Vector3d original_residual;
  constraint->CalcResidual(beluga_estimation::SE2d(), original_residual);
  EXPECT_FALSE(original_residual.isMuchSmallerThan(1.0, 1e-9));
}

TEST_F(OdomProbeTest, VelocityTransformIsApplied) {
  auto config = MakeConfig();
  config.base_frame = "map";
  config.use_pose = false;
  config.twist_mask = {true, true, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // The twist lives in the body frame ("base_link"); expect a 90° rotation
  // from "base_link" to the reference frame "map".
  const beluga_estimation::SE3d transform(
      beluga_estimation::SO3d(
          beluga_estimation::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, beluga_estimation::Vector3d::UnitZ()))),
      beluga_estimation::Vector3d::Zero());
  EXPECT_CALL(*middleware_->transform, LookupTransform(Eq("map"), Eq("base_link"), _, _)).WillOnce(Return(transform));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  const beluga_estimation::Vector3d linear_velocity(1.0, 0.0, 0.0);  // Along the body x-axis.
  captured_callback_(std::make_shared<const MockOdometry>(
      timestamp, beluga_estimation::SE2d{}, beluga_estimation::Matrix3d::Identity() * 0.01,
      beluga_estimation::Matrix3d::Identity() * 0.01, linear_velocity));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // The rotated measurement must be (0, 1, 0): the adjoint rotates both the
  // linear and the angular components.
  Identifier constraint_id{"odom/velocity/measurement", timestamp};
  auto constraint_result = ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint_result);
  auto constraint = Unwrap(constraint_result);
  EXPECT_NE(constraint, nullptr);

  const beluga_estimation::Vector3d expected_mean(0.0, 1.0, 0.0);
  beluga_estimation::VectorXd residual(3);
  constraint->CalcResidual(expected_mean, residual);
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));

  // The untransformed measurement is not consistent anymore.
  beluga_estimation::VectorXd original_residual(3);
  constraint->CalcResidual(beluga_estimation::Vector3d(1.0, 0.0, 0.0), original_residual);
  EXPECT_FALSE(original_residual.isMuchSmallerThan(1.0, 1e-9));
}

TEST_F(OdomProbeTest, TransformLookupFailureReturnsBusy) {
  auto config = MakeConfig();
  config.world_frame = "map";
  config.transform_timeout = std::chrono::nanoseconds::zero();
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // A missing transform is a transient condition, so the operation reports BUSY
  // for the filter loop to absorb instead of failing the whole pipeline.
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).WillOnce(Return(std::nullopt));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);

  EXPECT_EQ(probe.Execute(ctx, budget_).error(), Error::kBusy);

  // The lookup happens before the network is touched, so nothing is half-applied.
  EXPECT_EQ(ctx.dbn.GetVariable(Identifier{config.pose_variable_name, timestamp}), nullptr);
}

TEST_F(OdomProbeTest, NoTransformLookupWhenReferenceFrameEmpty) {
  auto config = MakeConfig();
  config.world_frame = "";  // no frame transformation
  config.twist_mask = {true, true, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // LookupTransform should never be called when reference_frame is empty.
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(0);

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // Both pose and velocity should exist despite no transform being applied.
  Identifier pose_id{std::string(OdomProbe::kPoseName), timestamp};
  EXPECT_NE(ctx.dbn.GetVariable(pose_id), nullptr);
  Identifier velocity_id{std::string(OdomProbe::kVelocityName), timestamp};
  auto velocity = ctx.dbn.GetVariable<EuclideanVariable_<double>>(velocity_id);
  ASSERT_TRUE(velocity);
  auto velocity_ptr = Unwrap(velocity);
  EXPECT_NE(velocity_ptr, nullptr);
  EXPECT_EQ(velocity_ptr->Size(), 3u);
}

// =============================================================================
// Parameter Loading Tests
// =============================================================================

TEST_F(OdomProbeTest, NoTransformLookupsWhenAllMeasurementsDisabled) {
  auto config = MakeConfig();
  config.use_pose = false;
  config.twist_mask = {false, false, false};
  config.world_frame = "map";
  config.base_frame = "map";
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(0);

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);

  ASSERT_TRUE(probe.Execute(ctx, budget_));
  EXPECT_EQ(ctx.dbn.GetVariable(Identifier{std::string(OdomProbe::kPoseName), timestamp}), nullptr);
  EXPECT_EQ(ctx.dbn.GetVariable(Identifier{std::string(OdomProbe::kVelocityName), timestamp}), nullptr);
}

TEST_F(OdomProbeTest, TransformTimeoutConversion) {
  ExpectDefaultParams(0.25);

  auto config = OdomProbe::Configuration::LoadFromParams(middleware_, "test.");
  // 0.25 seconds = 250,000,000 nanoseconds.
  EXPECT_EQ(config.transform_timeout.count(), 250000000);
}

// =============================================================================
// Covariance Tests
// =============================================================================

TEST_F(OdomProbeTest, PoseCovarianceIsAssembledFromMessage) {
  // Run once with no reference frame and once with a nontrivial transform:
  // the right-trivialized (body-frame) covariance is invariant under left
  // composition with a fixed transform.
  for (const auto world_frame : {std::string{""}, std::string{"map"}}) {
    auto config = MakeConfig();
    config.world_frame = world_frame;
    OdomProbe probe(middleware_, config);
    Context_<double> ctx;
    SetUpSubscriber();

    if (!world_frame.empty()) {
      const beluga_estimation::SE3d transform(
          beluga_estimation::SO3d(
              beluga_estimation::Quaterniond(Eigen::AngleAxisd(0.3, beluga_estimation::Vector3d::UnitZ()))),
          beluga_estimation::Vector3d(1.0, 2.0, 0.0));
      EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).WillOnce(Return(transform));
    }

    ASSERT_TRUE(probe.Initialize(budget_));

    const auto timestamp = TimePoint{std::chrono::seconds(100)};
    const beluga_estimation::Matrix3d position_covariance = beluga_estimation::Vector3d(0.1, 0.2, 0.3).asDiagonal();
    const beluga_estimation::Matrix3d orientation_covariance = beluga_estimation::Vector3d(0.4, 0.5, 0.6).asDiagonal();
    captured_callback_(std::make_shared<const MockOdometry>(
        timestamp, beluga_estimation::SE2d(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 2.0)),
        beluga_estimation::Matrix3d::Identity() * 0.01, beluga_estimation::Matrix3d::Identity() * 0.01,
        beluga_estimation::Vector3d::Zero(), beluga_estimation::Vector3d::Zero(), position_covariance,
        orientation_covariance));

    ASSERT_TRUE(probe.Execute(ctx, budget_));

    Identifier constraint_id{"odom/pose/measurement", timestamp};
    auto constraint_result = ctx.dbn.GetConstraint<SE2NormalPoseLikelihoodConstraint_<double>>(constraint_id);
    ASSERT_TRUE(constraint_result);
    auto constraint = Unwrap(constraint_result);
    EXPECT_NE(constraint, nullptr);

    // The SE(2) covariance is the [x, y, yaw] block of the pose covariance,
    // unchanged by the world-frame transform (right-trivialized convention).
    beluga_estimation::Matrix6d full_covariance = beluga_estimation::Matrix6d::Zero();
    full_covariance.topLeftCorner<3, 3>() = position_covariance;
    full_covariance.bottomRightCorner<3, 3>() = orientation_covariance;
    const beluga_estimation::Matrix3d expected_covariance = beluga_estimation::ToSE2Covariance(full_covariance);

    beluga_estimation::Matrix3d actual_covariance;
    constraint->CalcCovariance(actual_covariance);
    EXPECT_TRUE(actual_covariance.isApprox(expected_covariance, 1e-12));
  }
}

TEST_F(OdomProbeTest, PoseCovarianceTransformedByBaseFrameTransform) {
  auto config = MakeConfig();
  config.base_frame = "map";
  config.twist_mask = {true, true, true};  // the base-frame transform lookup is gated on this
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // Non-trivial transform from the body frame Bo to the base frame B.
  const beluga_estimation::SE3d transform(
      beluga_estimation::SO3d(
          beluga_estimation::Quaterniond(Eigen::AngleAxisd(0.3, beluga_estimation::Vector3d::UnitZ()))),
      beluga_estimation::Vector3d(1.0, 2.0, 0.0));
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).WillOnce(Return(transform));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  const beluga_estimation::Matrix3d position_covariance = beluga_estimation::Vector3d(0.1, 0.2, 0.3).asDiagonal();
  const beluga_estimation::Matrix3d orientation_covariance = beluga_estimation::Vector3d(0.4, 0.5, 0.6).asDiagonal();
  captured_callback_(std::make_shared<const MockOdometry>(
      timestamp, beluga_estimation::SE2d(beluga_estimation::SO2d(0.0), beluga_estimation::Vector2d(1.0, 2.0)),
      beluga_estimation::Matrix3d::Identity() * 0.01, beluga_estimation::Matrix3d::Identity() * 0.01,
      beluga_estimation::Vector3d::Zero(), beluga_estimation::Vector3d::Zero(), position_covariance,
      orientation_covariance));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier constraint_id{"odom/pose/measurement", timestamp};
  auto constraint_result = ctx.dbn.GetConstraint<SE2NormalPoseLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint_result);
  auto constraint = Unwrap(constraint_result);
  EXPECT_NE(constraint, nullptr);

  // The SE(2) covariance is the [x, y, yaw] block of the covariance
  // transformed by Ad(X_BBo) (right-trivialized convention).
  beluga_estimation::Matrix6d full_covariance = beluga_estimation::Matrix6d::Zero();
  full_covariance.topLeftCorner<3, 3>() = position_covariance;
  full_covariance.bottomRightCorner<3, 3>() = orientation_covariance;
  const auto Ad = transform.Adj();
  full_covariance = Ad * full_covariance * Ad.transpose();
  const beluga_estimation::Matrix3d expected_covariance = beluga_estimation::ToSE2Covariance(full_covariance);

  beluga_estimation::Matrix3d actual_covariance;
  constraint->CalcCovariance(actual_covariance);
  EXPECT_TRUE(actual_covariance.isApprox(expected_covariance, 1e-12));
}

TEST_F(OdomProbeTest, PoseCovariancePreservesCrossTerms) {
  auto config = MakeConfig();
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};

  // Full 6x6 pose covariance with cross-terms between translation and yaw.
  // The cross terms are small enough to keep the covariance positive definite.
  beluga_estimation::Matrix6d pose_covariance = beluga_estimation::Matrix6d::Zero();
  pose_covariance(0, 0) = 0.1;  // x
  pose_covariance(1, 1) = 0.2;  // y
  pose_covariance(5, 5) = 0.6;  // yaw
  pose_covariance(0, 5) = 0.05;
  pose_covariance(1, 5) = 0.05;
  pose_covariance(5, 0) = 0.05;
  pose_covariance(5, 1) = 0.05;
  const beluga_estimation::Matrix6d twist_covariance = beluga_estimation::Matrix6d::Identity() * 0.01;

  captured_callback_(std::make_shared<const MockOdometry>(
      MockOdometry::WithFullCovariance(timestamp, beluga_estimation::SE2d{}, pose_covariance, twist_covariance)));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // The SE(2) covariance must preserve the translation-yaw cross terms.
  Identifier constraint_id{"odom/pose/measurement", timestamp};
  auto constraint_result = ctx.dbn.GetConstraint<SE2NormalPoseLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint_result);
  auto constraint = Unwrap(constraint_result);
  EXPECT_NE(constraint, nullptr);

  beluga_estimation::Matrix3d expected_covariance = beluga_estimation::Matrix3d::Zero();
  expected_covariance(0, 0) = 0.1;
  expected_covariance(1, 1) = 0.2;
  expected_covariance(2, 2) = 0.6;
  expected_covariance(0, 2) = 0.05;
  expected_covariance(1, 2) = 0.05;
  expected_covariance(2, 0) = 0.05;
  expected_covariance(2, 1) = 0.05;

  beluga_estimation::Matrix3d actual_covariance;
  constraint->CalcCovariance(actual_covariance);
  EXPECT_TRUE(actual_covariance.isApprox(expected_covariance, 1e-12));
}

TEST_F(OdomProbeTest, VelocityTransformIncludesLeverArmAndCrossCovariance) {
  auto config = MakeConfig();
  config.base_frame = "map";
  config.use_pose = false;
  config.twist_mask = {true, true, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // Rotation and a non-zero lever arm.
  const beluga_estimation::SE3d transform(
      beluga_estimation::SO3d(
          beluga_estimation::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, beluga_estimation::Vector3d::UnitZ()))),
      beluga_estimation::Vector3d(0.5, 0.1, -0.2));
  EXPECT_CALL(*middleware_->transform, LookupTransform(Eq("map"), Eq("base_link"), _, _)).WillOnce(Return(transform));

  ASSERT_TRUE(probe.Initialize(budget_));

  const beluga_estimation::Vector3d linear_velocity(1.0, 0.0, 0.0);
  const beluga_estimation::Vector3d angular_velocity(0.0, 0.0, 0.5);
  const beluga_estimation::Matrix3d Svv = beluga_estimation::Vector3d(0.1, 0.2, 0.3).asDiagonal();
  const beluga_estimation::Matrix3d Sww = beluga_estimation::Vector3d(0.4, 0.5, 0.6).asDiagonal();
  const beluga_estimation::Matrix3d Sx = 0.05 * beluga_estimation::Matrix3d::Identity();
  beluga_estimation::Matrix6d twist_covariance = beluga_estimation::Matrix6d::Zero();
  twist_covariance.topLeftCorner<3, 3>() = Svv;
  twist_covariance.topRightCorner<3, 3>() = Sx;
  twist_covariance.bottomLeftCorner<3, 3>() = Sx.transpose();
  twist_covariance.bottomRightCorner<3, 3>() = Sww;

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  captured_callback_(std::make_shared<const MockOdometry>(MockOdometry::WithFullCovariance(
      timestamp, beluga_estimation::SE2d{}, beluga_estimation::Matrix6d::Identity(), twist_covariance, linear_velocity,
      angular_velocity)));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // Independent expectation: v' = R v + p x (R w) and w' = R w.
  const beluga_estimation::Matrix3d R = transform.rotationMatrix();
  const beluga_estimation::Vector3d p = transform.translation();
  const beluga_estimation::Vector3d expected_linear = R * linear_velocity + p.cross(R * angular_velocity);
  const beluga_estimation::Vector3d expected_angular = R * angular_velocity;

  // Independent expectation for the covariance: the adjoint built from R and
  // [p]x R, propagated through the full twist covariance.
  beluga_estimation::Matrix6d Ad = beluga_estimation::Matrix6d::Zero();
  Ad.topLeftCorner<3, 3>() = R;
  Ad.topRightCorner<3, 3>() = ToCrossProductMatrix(p) * R;
  Ad.bottomRightCorner<3, 3>() = R;
  const beluga_estimation::Matrix6d expected_full = Ad * twist_covariance * Ad.transpose();

  constexpr std::array<std::size_t, 3> kPlanarIndices{0, 1, 5};
  beluga_estimation::Vector3d expected_mean;
  beluga_estimation::Matrix3d expected_covariance;
  for (std::size_t i = 0; i < kPlanarIndices.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(kPlanarIndices[i]);
    for (std::size_t j = 0; j < kPlanarIndices.size(); ++j) {
      expected_covariance(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
          expected_full(row, static_cast<Eigen::Index>(kPlanarIndices[j]));
    }
  }
  expected_mean << expected_linear(0), expected_linear(1), expected_angular(2);

  Identifier constraint_id{"odom/velocity/measurement", timestamp};
  auto constraint_result = ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint_result);
  auto constraint = Unwrap(constraint_result);
  ASSERT_NE(constraint, nullptr);

  beluga_estimation::VectorXd residual(3);
  constraint->CalcResidual(expected_mean, residual);
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));

  beluga_estimation::MatrixXd actual_covariance(3, 3);
  constraint->CalcCovariance(actual_covariance);
  EXPECT_TRUE(actual_covariance.isApprox(expected_covariance, 1e-12));
}

TEST_F(OdomProbeTest, VelocityCovarianceIsTransformed) {
  auto config = MakeConfig();
  config.base_frame = "map";
  config.use_pose = false;
  config.twist_mask = {true, true, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  const beluga_estimation::SE3d transform(
      beluga_estimation::SO3d(
          beluga_estimation::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, beluga_estimation::Vector3d::UnitZ()))),
      beluga_estimation::Vector3d::Zero());
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).WillOnce(Return(transform));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  const beluga_estimation::Matrix3d linear_velocity_covariance =
      beluga_estimation::Vector3d(0.1, 0.2, 0.3).asDiagonal();
  const beluga_estimation::Matrix3d angular_velocity_covariance = beluga_estimation::Matrix3d::Identity() * 0.4;
  captured_callback_(std::make_shared<const MockOdometry>(
      timestamp, beluga_estimation::SE2d{}, linear_velocity_covariance, angular_velocity_covariance,
      beluga_estimation::Vector3d(1.0, 0.0, 0.0)));

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier constraint_id{"odom/velocity/measurement", timestamp};
  auto constraint_result = ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint_result);
  auto constraint = Unwrap(constraint_result);
  EXPECT_NE(constraint, nullptr);

  // The adjoint applies R(π/2) to the twist: the planar block is
  // [R * diag(0.1, 0.2) * Rᵀ, 0; 0, 0.4].
  const beluga_estimation::Matrix2d R = Eigen::Rotation2D<double>(M_PI / 2.0).toRotationMatrix().cast<double>();
  beluga_estimation::Matrix3d expected_covariance = beluga_estimation::Matrix3d::Zero();
  expected_covariance.topLeftCorner<2, 2>() = R * beluga_estimation::Vector2d(0.1, 0.2).asDiagonal() * R.transpose();
  expected_covariance(2, 2) = 0.4;

  beluga_estimation::MatrixXd actual_covariance(3, 3);
  constraint->CalcCovariance(actual_covariance);
  EXPECT_TRUE(actual_covariance.isApprox(expected_covariance, 1e-12));
}

// =============================================================================
// Duplicate and Multi-Message Tests
// =============================================================================

TEST_F(OdomProbeTest, DuplicatePoseConstraintReturnsError) {
  OdomProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  // A message is processed cleanly...
  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);
  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // ...but a second message sharing the same timestamp cannot add a duplicate
  // pose constraint, so the probe reports an error.
  InjectPoseMessage(timestamp);
  EXPECT_EQ(probe.Execute(ctx, budget_).error(), Error::kFailure);
}

TEST_F(OdomProbeTest, ProcessesMultipleMessagesInOneExecute) {
  OdomProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto t1 = TimePoint{std::chrono::seconds(100)};
  const auto t2 = TimePoint{std::chrono::seconds(101)};
  InjectPoseMessage(t1);
  InjectPoseMessage(t2);

  // A single Execute must drain the whole queue.
  ASSERT_TRUE(probe.Execute(ctx, budget_));
  EXPECT_EQ(probe.Execute(ctx, budget_).error(), Error::kBusy);

  Identifier pose_id_1{std::string(OdomProbe::kPoseName), t1};
  EXPECT_NE(ctx.dbn.GetVariable(pose_id_1), nullptr);
  Identifier pose_id_2{std::string(OdomProbe::kPoseName), t2};
  EXPECT_NE(ctx.dbn.GetVariable(pose_id_2), nullptr);
}

TEST_F(OdomProbeTest, NoWorldTransformLookupForVelocityOnly) {
  auto config = MakeConfig();
  config.world_frame = "map";
  config.use_pose = false;
  config.twist_mask = {true, true, true};
  OdomProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // The yaw rate is invariant under planar rotations: no transform lookup.
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(0);

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier variable_id{std::string(OdomProbe::kVelocityName), timestamp};
  EXPECT_NE(ctx.dbn.GetVariable(variable_id), nullptr);
}

// =============================================================================
// Budget and Scalar Type Tests
// =============================================================================

TEST_F(OdomProbeTest, BudgetTimeoutStopsProcessing) {
  OdomProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(ComputeBudget::Unlimited()));

  // A budget whose deadline is already in the past relative to the mock clock.
  ComputeBudget budget{TimePoint{} - std::chrono::seconds(1), {}};

  const auto t1 = TimePoint{std::chrono::seconds(100)};
  const auto t2 = TimePoint{std::chrono::seconds(101)};
  InjectPoseMessage(t1);
  InjectPoseMessage(t2);

  // The first message is processed, then the budget check times out.
  EXPECT_EQ(probe.Execute(ctx, budget).error(), Error::kTimeout);

  Identifier pose_id_1{std::string(OdomProbe::kPoseName), t1};
  EXPECT_NE(ctx.dbn.GetVariable(pose_id_1), nullptr);
  Identifier pose_id_2{std::string(OdomProbe::kPoseName), t2};
  EXPECT_EQ(ctx.dbn.GetVariable(pose_id_2), nullptr);
}

TEST_F(OdomProbeTest, FloatInstantiationProcessesMessage) {
  OdomProbe_<float>::Configuration config;
  config.device_name = "odom";
  config.topic_name = "odom";
  config.use_pose = true;
  config.twist_mask = {false, false, false};

  OdomProbe_<float> probe(middleware_, config);
  Context_<float> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectPoseMessage(timestamp);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier variable_id{std::string(OdomProbe::kPoseName), timestamp};
  auto variable_result = ctx.dbn.GetVariable<SE2Variable_<float>>(variable_id);
  ASSERT_TRUE(variable_result);
  EXPECT_NE(Unwrap(variable_result), nullptr);
}
