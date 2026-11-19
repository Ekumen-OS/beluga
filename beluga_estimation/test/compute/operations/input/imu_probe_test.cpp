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
#include <beluga_estimation/compute/operations/input/imu_probe.hpp>
#include <beluga_estimation/constraints/normal_likelihood_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/variable.hpp>
#include <beluga_estimation/mocks/mock_clock.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/mocks/mock_subscriber.hpp>
#include <beluga_estimation/msg/imu.hpp>

using namespace beluga_estimation;
using ::testing::_;
using ::testing::Return;

namespace {

// -----------------------------------------------------------------------------
// Test-only IMU message and header implementations.
// -----------------------------------------------------------------------------

/// A minimal IMU message implementation for testing.
class TestImu : public beluga_estimation::msg::Imu {
 public:
  TestImu(
      beluga_estimation::TimePoint stamp,
      beluga_estimation::Vector3d angular_velocity,
      beluga_estimation::Matrix3d angular_velocity_covariance)
      : TestImu(
            stamp,
            angular_velocity,
            angular_velocity_covariance,
            beluga_estimation::Vector3d::Zero(),
            beluga_estimation::Matrix3d::Identity()) {}

  TestImu(
      beluga_estimation::TimePoint stamp,
      beluga_estimation::Vector3d angular_velocity,
      beluga_estimation::Matrix3d angular_velocity_covariance,
      beluga_estimation::Vector3d linear_acceleration,
      beluga_estimation::Matrix3d linear_acceleration_covariance)
      : timestamp_(stamp),
        angular_velocity_(angular_velocity),
        linear_acceleration_(linear_acceleration),
        angular_velocity_covariance_(angular_velocity_covariance),
        linear_acceleration_covariance_(linear_acceleration_covariance),
        orientation_covariance_(beluga_estimation::Matrix3d::Identity()) {
    orientation_.coeffs() << 1.0, 0.0, 0.0, 0.0;
  }

  beluga_estimation::TimePoint GetTimestamp() const final { return timestamp_; }

  std::string_view GetFrameId() const final { return frame_id_; }

  QuaternionView GetOrientation() const final { return QuaternionView(orientation_.coeffs().data()); }

  Vector3View GetAngularVelocity() const final { return Vector3View(angular_velocity_.data()); }

  Vector3View GetLinearAcceleration() const final { return Vector3View(linear_acceleration_.data()); }

  Matrix3View GetOrientationCovariance() const final { return Matrix3View(orientation_covariance_.data()); }

  Matrix3View GetAngularVelocityCovariance() const final { return Matrix3View(angular_velocity_covariance_.data()); }

  Matrix3View GetLinearAccelerationCovariance() const final {
    return Matrix3View(linear_acceleration_covariance_.data());
  }

  const void* GetNativeHandle() const final { return &dummy_; }

 private:
  beluga_estimation::TimePoint timestamp_;
  std::string_view frame_id_{"imu_link"};
  beluga_estimation::Vector3d angular_velocity_;
  beluga_estimation::Vector3d linear_acceleration_ = beluga_estimation::Vector3d::Zero();
  beluga_estimation::Matrix3d angular_velocity_covariance_;
  beluga_estimation::Matrix3d linear_acceleration_covariance_ = beluga_estimation::Matrix3d::Identity();
  beluga_estimation::Matrix3d orientation_covariance_ = beluga_estimation::Matrix3d::Identity();
  Eigen::Quaterniond orientation_;
  int dummy_ = 0;
};

}  // namespace

class IMUProbeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    middleware_ = std::make_shared<MockMiddleware>();
    clock_ = std::make_shared<MockClock>();

    ON_CALL(*middleware_, GetClockInterface()).WillByDefault(Return(clock_.get()));
    EXPECT_CALL(*middleware_, GetClockInterface()).Times(::testing::AnyNumber());
  }

  // A configuration that processes the angular velocity channel only.
  static IMUProbe::Configuration MakeConfig() {
    IMUProbe::Configuration config;
    config.device_name = "imu";
    config.topic_name = "/imu/data";
    config.use_2d_mode = false;
    config.angular_velocity_mask = {true, true, true};
    config.linear_acceleration_mask = {false, false, false};
    return config;
  }

  // Stubs the parameter declarations LoadFromParams reads, with the given
  // transform_timeout value (in seconds) and variable names.
  void ExpectDefaultParams(
      double transform_timeout,
      const std::string& velocity_variable_name = "velocity",
      const std::string& acceleration_variable_name = "acceleration") {
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.device_name", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("imu")}));
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.topic_name", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("/imu/data")}));
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.body_frame", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::string("")}));
    EXPECT_CALL(
        *middleware_->parameters, DeclareParameter("test.transform_timeout", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{transform_timeout}));
    EXPECT_CALL(
        *middleware_->parameters, DeclareParameter("test.velocity_variable_name", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{velocity_variable_name}));
    EXPECT_CALL(
        *middleware_->parameters,
        DeclareParameter("test.acceleration_variable_name", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{acceleration_variable_name}));
    EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.use_2d_mode", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{false}));
    EXPECT_CALL(
        *middleware_->parameters, DeclareParameter("test.angular_velocity_mask", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::vector<bool>{true, true, true}}));
    EXPECT_CALL(
        *middleware_->parameters,
        DeclareParameter("test.linear_acceleration_mask", testing::An<const ParameterValue&>()))
        .WillOnce(Return(ParameterValue{std::vector<bool>{false, false, false}}));
  }

  // Sets up the mock subscriber and captures the callback handed to it.
  void SetUpSubscriber() {
    EXPECT_CALL(*middleware_->topics, CreateSubscriberImpl(_, _, _))
        .WillOnce(testing::DoAll(
            testing::SaveArg<2>(&captured_callback_),
            testing::Return(std::make_shared<MockSubscriber<beluga_estimation::msg::Imu>>())));
  }

  // Injects a test IMU message through the captured subscriber callback.
  void InjectMessage(
      beluga_estimation::TimePoint stamp,
      beluga_estimation::Vector3d angular_velocity,
      beluga_estimation::Matrix3d covariance) {
    captured_callback_(std::make_shared<const TestImu>(stamp, angular_velocity, covariance));
  }

  // Injects a test IMU message with explicit linear acceleration through the captured subscriber callback.
  void InjectMessage(
      beluga_estimation::TimePoint stamp,
      beluga_estimation::Vector3d angular_velocity,
      beluga_estimation::Matrix3d angular_velocity_covariance,
      beluga_estimation::Vector3d linear_acceleration,
      beluga_estimation::Matrix3d linear_acceleration_covariance) {
    captured_callback_(std::make_shared<const TestImu>(
        stamp, angular_velocity, angular_velocity_covariance, linear_acceleration, linear_acceleration_covariance));
  }

  std::shared_ptr<MockMiddleware> middleware_;
  std::shared_ptr<MockClock> clock_;
  beluga_estimation::SubscriberCallback<beluga_estimation::Message> captured_callback_;
  ComputeBudget budget_{ComputeBudget::Unlimited()};
};

// =============================================================================
// Basic Compilation and Type Tests
// =============================================================================

TEST_F(IMUProbeTest, CanInstantiateConfigurationStruct) {
  IMUProbe::Configuration config;
  config.device_name = "test_imu";
  config.topic_name = "/test/imu";
  config.body_frame = "";
  config.use_2d_mode = false;
  config.angular_velocity_mask = {true, true, true};
  config.linear_acceleration_mask = {false, false, false};

  EXPECT_EQ(config.device_name, "test_imu");
  EXPECT_EQ(config.topic_name, "/test/imu");
  EXPECT_EQ(config.body_frame, "");
  EXPECT_EQ(config.transform_timeout.count(), 0);
  EXPECT_FALSE(config.use_2d_mode);
  EXPECT_EQ(config.angular_velocity_mask, std::vector<bool>({true, true, true}));
  EXPECT_EQ(config.linear_acceleration_mask, std::vector<bool>({false, false, false}));
}

TEST_F(IMUProbeTest, ConfigurationDefaults) {
  // Default-constructed configurations must match the LoadFromParams defaults.
  IMUProbe::Configuration config;
  EXPECT_FALSE(config.use_2d_mode);
  EXPECT_EQ(config.angular_velocity_mask, std::vector<bool>({true, true, true}));
  EXPECT_EQ(config.linear_acceleration_mask, std::vector<bool>({false, false, false}));
  EXPECT_EQ(config.transform_timeout.count(), 0);
}

TEST_F(IMUProbeTest, ConstructionWithExplicitConfig) {
  EXPECT_NO_THROW({ IMUProbe probe(middleware_, MakeConfig()); });
}

TEST_F(IMUProbeTest, ConstructionFromParams) {
  ExpectDefaultParams(0.0);
  EXPECT_NO_THROW({ IMUProbe probe(middleware_, "test."); });
}

// =============================================================================
// Lifecycle Tests
// =============================================================================

TEST_F(IMUProbeTest, InitializeCreatesSubscription) {
  IMUProbe probe(middleware_, MakeConfig());

  // Initialize creates a subscriber for the configured topic.
  EXPECT_CALL(*middleware_->topics, CreateSubscriberImpl(_, _, _))
      .WillOnce(Return(std::make_shared<MockSubscriber<beluga_estimation::msg::Imu>>()));

  EXPECT_TRUE(probe.Initialize(budget_));
}

TEST_F(IMUProbeTest, FinalizeDestroysSubscription) {
  IMUProbe probe(middleware_, MakeConfig());

  EXPECT_CALL(*middleware_->topics, CreateSubscriberImpl(_, _, _))
      .WillOnce(Return(std::make_shared<MockSubscriber<beluga_estimation::msg::Imu>>()));

  ASSERT_TRUE(probe.Initialize(budget_));
  EXPECT_TRUE(probe.Finalize(budget_));
}

TEST_F(IMUProbeTest, ExecuteWithEmptyQueueReturnsBusy) {
  IMUProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;

  // With an empty message queue, Execute should return BUSY.
  EXPECT_EQ(probe.Execute(ctx, budget_).error(), Error::kBusy);
}

// =============================================================================
// Message Processing Tests
// =============================================================================

TEST_F(IMUProbeTest, ProcessAngularVelocityMessage) {
  IMUProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  // Create and inject a test IMU message.
  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());

  // Execute should process the message.
  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // In 3D mode the gyro constrains the angular slice of the 6D velocity variable.
  Identifier variable_id{std::string(IMUProbe::kVelocityName), timestamp};
  Identifier constraint_id{"imu/velocity/measurement", timestamp};
  auto variable = ctx.dbn.GetVariable<EuclideanVariable_<double>>(variable_id);
  ASSERT_TRUE(variable);
  auto variable_ptr = Unwrap(variable);
  ASSERT_NE(variable_ptr, nullptr);
  EXPECT_EQ(variable_ptr->Size(), 6u);
  auto constraint = ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint);
  ASSERT_NE(Unwrap(constraint), nullptr);
  EXPECT_EQ(Unwrap(constraint)->ResidualSize(), 3u);
}

TEST_F(IMUProbeTest, ProcessAngularVelocityMessage2DMode) {
  auto config = MakeConfig();
  config.use_2d_mode = true;
  config.angular_velocity_mask = {false, false, true};
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(
      timestamp, beluga_estimation::Vector3d(0.0, 0.0, 0.5),
      beluga_estimation::Matrix3d::Identity());  // ω_z only in 2D

  EXPECT_TRUE(probe.Execute(ctx, budget_));

  // In 2D mode the gyro constrains the angular slice of the 3D velocity variable.
  Identifier variable_id{std::string(IMUProbe::kVelocityName), timestamp};
  Identifier constraint_id{"imu/velocity/measurement", timestamp};
  auto variable = ctx.dbn.GetVariable<EuclideanVariable_<double>>(variable_id);
  ASSERT_TRUE(variable);
  auto variable_ptr = Unwrap(variable);
  ASSERT_NE(variable_ptr, nullptr);
  EXPECT_EQ(variable_ptr->Size(), 3u);
  auto constraint = ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint);
  ASSERT_NE(Unwrap(constraint), nullptr);
  EXPECT_EQ(Unwrap(constraint)->ResidualSize(), 1u);
}

TEST_F(IMUProbeTest, ProcessAngularVelocityAndAcceleration) {
  auto config = MakeConfig();
  config.linear_acceleration_mask = {true, true, true};
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity() * 0.01);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // Verify both variables were added to the DBN. In 3D mode acceleration
  // mirrors the 6D velocity variable.
  Identifier vel_id{std::string(IMUProbe::kVelocityName), timestamp};
  {
    auto velocity_result = ctx.dbn.GetVariable<EuclideanVariable_<double>>(vel_id);
    ASSERT_TRUE(velocity_result);
    auto velocity = Unwrap(velocity_result);
    ASSERT_NE(velocity, nullptr);
    EXPECT_EQ(velocity->Size(), 6u);
  }

  Identifier acc_id{std::string(IMUProbe::kAccelerationName), timestamp};
  {
    auto acceleration_result = ctx.dbn.GetVariable<EuclideanVariable_<double>>(acc_id);
    ASSERT_TRUE(acceleration_result);
    auto acceleration = Unwrap(acceleration_result);
    ASSERT_NE(acceleration, nullptr);
    EXPECT_EQ(acceleration->Size(), 6u);
  }
}

TEST_F(IMUProbeTest, VelocityMaskSelectsAngularCoordinates) {
  auto config = MakeConfig();
  config.angular_velocity_mask = {false, true, false};
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier constraint_id{"imu/velocity/measurement", timestamp};
  auto constraint = Unwrap(ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id));
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->ResidualSize(), 1u);

  // Only wy, coordinate 4 of the 6D velocity variable, is observed.
  std::vector<Vector1d> empty;
  std::vector<beluga_estimation::Vector6d> state = {{999.0, 999.0, 999.0, 999.0, 0.2, 999.0}};
  beluga_estimation::VectorXd residual(1);
  constraint->CalcResidual(to_view(empty), to_view(state), to_view(residual));
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));
}

TEST_F(IMUProbeTest, AccelerationMaskSelectsLinearCoordinates) {
  auto config = MakeConfig();
  config.linear_acceleration_mask = {true, false, true};
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(
      timestamp, beluga_estimation::Vector3d::Zero(), beluga_estimation::Matrix3d::Identity(),
      beluga_estimation::Vector3d(0.5, 0.2, -0.3), beluga_estimation::Matrix3d::Identity());

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier constraint_id{"imu/acceleration/measurement", timestamp};
  auto constraint = Unwrap(ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id));
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->ResidualSize(), 2u);

  // ax and az are observed; ay is skipped even though it lies between them.
  std::vector<Vector1d> empty;
  std::vector<beluga_estimation::Vector6d> state = {{0.5, 999.0, -0.3, 999.0, 999.0, 999.0}};
  beluga_estimation::VectorXd residual(2);
  constraint->CalcResidual(to_view(empty), to_view(state), to_view(residual));
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));
}

TEST_F(IMUProbeTest, AccelerationMask2DUsesPlanarLinearCoordinates) {
  auto config = MakeConfig();
  config.use_2d_mode = true;
  config.angular_velocity_mask = {false, false, false};
  config.linear_acceleration_mask = {true, false, false};
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(
      timestamp, beluga_estimation::Vector3d::Zero(), beluga_estimation::Matrix3d::Identity(),
      beluga_estimation::Vector3d(0.5, 0.2, -0.3), beluga_estimation::Matrix3d::Identity());

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier variable_id{std::string(IMUProbe::kAccelerationName), timestamp};
  {
    auto acceleration_result = ctx.dbn.GetVariable<EuclideanVariable_<double>>(variable_id);
    ASSERT_TRUE(acceleration_result);
    auto acceleration = Unwrap(acceleration_result);
    ASSERT_NE(acceleration, nullptr);
    EXPECT_EQ(acceleration->Size(), 3u);
  }

  Identifier constraint_id{"imu/acceleration/measurement", timestamp};
  auto constraint = Unwrap(ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id));
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->ResidualSize(), 1u);

  // Only ax, coordinate 0 of the 3D acceleration variable, is observed.
  std::vector<Vector1d> empty;
  std::vector<Vector3d> state = {{0.5, 999.0, 999.0}};
  beluga_estimation::VectorXd residual(1);
  constraint->CalcResidual(to_view(empty), to_view(state), to_view(residual));
  EXPECT_TRUE(residual.isMuchSmallerThan(1.0, 1e-9));
}

TEST_F(IMUProbeTest, UsesConfiguredVariableNames) {
  auto config = MakeConfig();
  config.linear_acceleration_mask = {true, true, true};
  config.velocity_variable_name = "body_rate";
  config.acceleration_variable_name = "body_accel";
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  {
    auto rate_result = ctx.dbn.GetVariable<EuclideanVariable_<double>>(Identifier{"body_rate", timestamp});
    ASSERT_TRUE(rate_result);
    auto rate = Unwrap(rate_result);
    ASSERT_NE(rate, nullptr);
    EXPECT_EQ(rate->Size(), 6u);
  }
  {
    auto body_acceleration_result =
        ctx.dbn.GetVariable<EuclideanVariable_<double>>(Identifier{"body_accel", timestamp});
    ASSERT_TRUE(body_acceleration_result);
    auto body_acceleration = Unwrap(body_acceleration_result);
    ASSERT_NE(body_acceleration, nullptr);
    EXPECT_EQ(body_acceleration->Size(), 6u);
  }
  ASSERT_EQ(ctx.dbn.GetVariable(Identifier{std::string(IMUProbe::kVelocityName), timestamp}), nullptr);

  // Constraint names derive from the configured variable names, not the defaults.
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"imu/body_rate/measurement", timestamp}), nullptr);
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"imu/body_accel/measurement", timestamp}), nullptr);
  EXPECT_EQ(ctx.dbn.GetConstraint(Identifier{"imu/velocity/measurement", timestamp}), nullptr);
  EXPECT_EQ(ctx.dbn.GetConstraint(Identifier{"imu/acceleration/measurement", timestamp}), nullptr);
}

TEST_F(IMUProbeTest, ProcessesMultipleMessagesInOneExecute) {
  IMUProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;
  SetUpSubscriber();

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto t1 = TimePoint{std::chrono::seconds(100)};
  const auto t2 = TimePoint{std::chrono::seconds(101)};
  const auto covariance = beluga_estimation::Matrix3d::Identity();
  InjectMessage(t1, beluga_estimation::Vector3d(0.1, 0.2, 0.3), covariance);
  InjectMessage(t2, beluga_estimation::Vector3d(0.1, 0.2, 0.3), covariance);

  // A single Execute must drain the whole queue.
  ASSERT_TRUE(probe.Execute(ctx, budget_));
  EXPECT_EQ(probe.Execute(ctx, budget_).error(), Error::kBusy);

  Identifier ang_vel_id_1{std::string(IMUProbe::kVelocityName), t1};
  EXPECT_NE(ctx.dbn.GetVariable(ang_vel_id_1), nullptr);
  Identifier ang_vel_id_2{std::string(IMUProbe::kVelocityName), t2};
  EXPECT_NE(ctx.dbn.GetVariable(ang_vel_id_2), nullptr);
}

TEST_F(IMUProbeTest, EmitsDebugSummaryAfterProcessingMessages) {
  IMUProbe probe(middleware_, MakeConfig());
  Context_<double> ctx;
  SetUpSubscriber();

  std::vector<std::string> debug_messages;
  ON_CALL(*middleware_->logger, Log(LogLevel::kDebug, _))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });
  EXPECT_CALL(*middleware_->logger, Log(LogLevel::kDebug, _)).Times(::testing::AnyNumber());

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto covariance = beluga_estimation::Matrix3d::Identity();
  InjectMessage(TimePoint{std::chrono::seconds(100)}, beluga_estimation::Vector3d(0.1, 0.2, 0.3), covariance);
  InjectMessage(TimePoint{std::chrono::seconds(101)}, beluga_estimation::Vector3d(0.1, 0.2, 0.3), covariance);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  EXPECT_THAT(debug_messages, ::testing::Contains("IMUProbe: processed 2 message(s) from 'imu' device"));
}

// =============================================================================
// Transform Lookup Tests
// =============================================================================

TEST_F(IMUProbeTest, SingleTransformLookupPerMessage) {
  auto config = MakeConfig();
  config.body_frame = "map";
  config.linear_acceleration_mask = {true, true, true};
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // Expect exactly one LookupTransform call (shared by both processors).
  const beluga_estimation::SE3d identity_se3{};  // Default-constructs to the identity.
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(1).WillOnce(Return(identity_se3));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // Both variables should exist.
  Identifier ang_vel_id{std::string(IMUProbe::kVelocityName), timestamp};
  EXPECT_NE(ctx.dbn.GetVariable(ang_vel_id), nullptr);
  Identifier lin_acc_id{std::string(IMUProbe::kAccelerationName), timestamp};
  EXPECT_NE(ctx.dbn.GetVariable(lin_acc_id), nullptr);
}

TEST_F(IMUProbeTest, TransformLookupFailureReturnsBusy) {
  auto config = MakeConfig();
  config.body_frame = "map";
  config.transform_timeout = std::chrono::nanoseconds::zero();
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // A missing transform is a transient condition, so the operation reports BUSY
  // for the filter loop to absorb instead of failing the whole pipeline.
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(1).WillOnce(Return(std::nullopt));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());

  EXPECT_EQ(probe.Execute(ctx, budget_).error(), Error::kBusy);

  // The lookup happens before the network is touched, so nothing is half-applied.
  EXPECT_EQ(ctx.dbn.GetVariable(Identifier{config.velocity_variable_name, timestamp}), nullptr);
}

TEST_F(IMUProbeTest, NoTransformLookupWhenReferenceFrameEmpty) {
  auto config = MakeConfig();
  config.body_frame = "";  // no frame transformation
  config.linear_acceleration_mask = {true, true, true};
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // LookupTransform should never be called when reference_frame is empty.
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(0);

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // Both variables should exist despite no transform being applied.
  Identifier ang_vel_id{std::string(IMUProbe::kVelocityName), timestamp};
  EXPECT_NE(ctx.dbn.GetVariable(ang_vel_id), nullptr);
  Identifier lin_acc_id{std::string(IMUProbe::kAccelerationName), timestamp};
  EXPECT_NE(ctx.dbn.GetVariable(lin_acc_id), nullptr);
}

// =============================================================================
// Lever Arm (Linear Acceleration Transform) Tests
// =============================================================================

TEST_F(IMUProbeTest, LinearAccelerationLeverArmTransform3D) {
  auto config = MakeConfig();
  config.linear_acceleration_mask = {true, true, true};
  config.body_frame = "base_link";
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // Non-identity transform: 90° rotation about Z and a lever arm offset.
  const beluga_estimation::SE3d transform{
      Eigen::Quaterniond{Eigen::AngleAxisd{M_PI / 2.0, Eigen::Vector3d::UnitZ()}}, Eigen::Vector3d{0.5, 0.1, -0.2}};
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(1).WillOnce(Return(transform));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  const beluga_estimation::Vector3d w(0.2, -0.3, 0.5);  // non-axis-aligned angular velocity (sensor frame)
  // Purely centripetal acceleration: a = w x (w x r), with r = R^T t the lever arm
  // in the sensor frame. After the lever arm correction the origin acceleration
  // must be zero: a_origin = a - w x (w x r) = 0.
  const beluga_estimation::Vector3d r = transform.rotationMatrix().transpose() * transform.translation();
  const beluga_estimation::Vector3d a = w.cross(w.cross(r));
  const beluga_estimation::Matrix3d Ew = beluga_estimation::Matrix3d::Identity() * 0.01;
  const beluga_estimation::Matrix3d Ea = beluga_estimation::Matrix3d::Identity() * 0.02;
  InjectMessage(timestamp, w, Ew, a, Ea);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  Identifier constraint_id{"imu/acceleration/measurement", timestamp};
  auto constraint_result = ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint_result);
  auto constraint = Unwrap(constraint_result);
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->ResidualSize(), 3u);

  // The transformed mean must be zero (the centripetal term is subtracted).
  // The typed API takes the full acceleration variable.
  beluga_estimation::VectorXd residual(3);
  constraint->CalcResidual(beluga_estimation::Vector6d::Zero(), residual);
  const beluga_estimation::Vector3d actual_mean = -residual;
  EXPECT_NEAR(actual_mean.norm(), 0.0, 1e-9);

  // Expected covariance: R * (Ea + Jw * Ew * Jw^T) * R^T, with Jw = [r]x[w]x - 2[w]x[r]x.
  const beluga_estimation::Matrix3d wx = ToCrossProductMatrix(w);
  const beluga_estimation::Matrix3d rx = ToCrossProductMatrix(r);
  const beluga_estimation::Matrix3d Jw = rx * wx - 2 * wx * rx;
  const beluga_estimation::Matrix3d expected_cov =
      transform.rotationMatrix() * (Ea + Jw * Ew * Jw.transpose()) * transform.rotationMatrix().transpose();
  beluga_estimation::MatrixXd actual_cov(3, 3);
  constraint->CalcCovariance(actual_cov);
  EXPECT_TRUE(actual_cov.isApprox(expected_cov, 1e-9));
}

TEST_F(IMUProbeTest, LinearAccelerationLeverArmTransform2D) {
  auto config = MakeConfig();
  config.angular_velocity_mask = {false, false, false};
  config.linear_acceleration_mask = {true, true, false};
  config.use_2d_mode = true;
  config.body_frame = "base_link";
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  // Non-identity transform: 90° rotation about Z and a lever arm offset.
  const beluga_estimation::SE3d transform{
      Eigen::Quaterniond{Eigen::AngleAxisd{M_PI / 2.0, Eigen::Vector3d::UnitZ()}}, Eigen::Vector3d{0.5, 0.1, -0.2}};
  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(1).WillOnce(Return(transform));

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  const beluga_estimation::Vector3d w(0.2, -0.3, 0.5);  // non-axis-aligned angular velocity (sensor frame)
  // Purely centripetal acceleration (see the 3D test for the rationale).
  const beluga_estimation::Vector3d r = transform.rotationMatrix().transpose() * transform.translation();
  const beluga_estimation::Vector3d a = w.cross(w.cross(r));
  const beluga_estimation::Matrix3d Ew = beluga_estimation::Matrix3d::Identity() * 0.01;
  const beluga_estimation::Matrix3d Ea = beluga_estimation::Matrix3d::Identity() * 0.02;
  InjectMessage(timestamp, w, Ew, a, Ea);

  ASSERT_TRUE(probe.Execute(ctx, budget_));

  // In 2D mode acceleration mirrors the 3D velocity variable, and the
  // accelerometer observes its linear slice.
  Identifier variable_id{std::string(IMUProbe::kAccelerationName), timestamp};
  {
    auto acceleration_result = ctx.dbn.GetVariable<EuclideanVariable_<double>>(variable_id);
    ASSERT_TRUE(acceleration_result);
    auto acceleration = Unwrap(acceleration_result);
    ASSERT_NE(acceleration, nullptr);
    EXPECT_EQ(acceleration->Size(), 3u);
  }

  Identifier constraint_id{"imu/acceleration/measurement", timestamp};
  auto constraint_result = ctx.dbn.GetConstraint<NormalLikelihoodConstraint_<double>>(constraint_id);
  ASSERT_TRUE(constraint_result);
  auto constraint = Unwrap(constraint_result);
  ASSERT_NE(constraint, nullptr);
  EXPECT_EQ(constraint->ResidualSize(), 2u);

  // The 2D mean (head<2> of the transformed 3D mean) must be zero.
  // The typed API takes the full acceleration variable.
  beluga_estimation::VectorXd residual(2);
  constraint->CalcResidual(beluga_estimation::Vector3d::Zero(), residual);
  const beluga_estimation::Vector2d actual_mean = -residual;
  EXPECT_NEAR(actual_mean.norm(), 0.0, 1e-9);

  // Expected 2D covariance: top-left 2x2 block of the transformed 3D covariance.
  const beluga_estimation::Matrix3d wx = ToCrossProductMatrix(w);
  const beluga_estimation::Matrix3d rx = ToCrossProductMatrix(r);
  const beluga_estimation::Matrix3d Jw = rx * wx - 2 * wx * rx;
  const beluga_estimation::Matrix3d expected_cov_3d =
      transform.rotationMatrix() * (Ea + Jw * Ew * Jw.transpose()) * transform.rotationMatrix().transpose();
  beluga_estimation::MatrixXd actual_cov(2, 2);
  constraint->CalcCovariance(actual_cov);
  EXPECT_TRUE(actual_cov.isApprox(expected_cov_3d.topLeftCorner<2, 2>(), 1e-9));
}

// =============================================================================
// Parameter Loading Tests
// =============================================================================

TEST_F(IMUProbeTest, NoTransformLookupWhenAllMeasurementsDisabled) {
  auto config = MakeConfig();
  config.angular_velocity_mask = {false, false, false};
  config.linear_acceleration_mask = {false, false, false};
  config.body_frame = "map";
  IMUProbe probe(middleware_, config);
  Context_<double> ctx;
  SetUpSubscriber();

  EXPECT_CALL(*middleware_->transform, LookupTransform(_, _, _, _)).Times(0);

  ASSERT_TRUE(probe.Initialize(budget_));

  const auto timestamp = TimePoint{std::chrono::seconds(100)};
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());

  ASSERT_TRUE(probe.Execute(ctx, budget_));
  EXPECT_EQ(ctx.dbn.GetVariable(Identifier{std::string(IMUProbe::kVelocityName), timestamp}), nullptr);
  EXPECT_EQ(ctx.dbn.GetVariable(Identifier{std::string(IMUProbe::kAccelerationName), timestamp}), nullptr);
}

TEST_F(IMUProbeTest, TransformTimeoutConversion) {
  ExpectDefaultParams(0.25);

  auto config = IMUProbe::Configuration::LoadFromParams(middleware_, "test.");
  // 0.25 seconds = 250,000,000 nanoseconds.
  EXPECT_EQ(config.transform_timeout.count(), 250000000);
}

TEST_F(IMUProbeTest, LoadFromParamsReadsVariableNames) {
  ExpectDefaultParams(0.0, "agent0/velocity", "agent0/acceleration");

  auto config = IMUProbe::Configuration::LoadFromParams(middleware_, "test.");
  EXPECT_EQ(config.velocity_variable_name, "agent0/velocity");
  EXPECT_EQ(config.acceleration_variable_name, "agent0/acceleration");
}

TEST_F(IMUProbeTest, MultiAgentProbesDoNotCollide) {
  auto agent0_config = MakeConfig();
  agent0_config.velocity_variable_name = "agent0/velocity";
  auto agent1_config = MakeConfig();
  agent1_config.velocity_variable_name = "agent1/velocity";

  // Both agents keep the default device name; only the variable names differ.
  IMUProbe agent0(middleware_, agent0_config);
  IMUProbe agent1(middleware_, agent1_config);

  Context_<double> ctx;
  const auto timestamp = TimePoint{std::chrono::seconds(100)};

  SetUpSubscriber();
  ASSERT_TRUE(agent0.Initialize(budget_));
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());
  ASSERT_TRUE(agent0.Execute(ctx, budget_));

  SetUpSubscriber();
  ASSERT_TRUE(agent1.Initialize(budget_));
  InjectMessage(timestamp, beluga_estimation::Vector3d(0.1, 0.2, 0.3), beluga_estimation::Matrix3d::Identity());
  ASSERT_TRUE(agent1.Execute(ctx, budget_));

  // A name clash would have made the second agent's insertions fail.
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"imu/agent0/velocity/measurement", timestamp}), nullptr);
  EXPECT_NE(ctx.dbn.GetConstraint(Identifier{"imu/agent1/velocity/measurement", timestamp}), nullptr);
  ASSERT_NE(ctx.dbn.GetVariable(Identifier{"agent0/velocity", timestamp}), nullptr);
  ASSERT_NE(ctx.dbn.GetVariable(Identifier{"agent1/velocity", timestamp}), nullptr);
}
