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

#include <gtest/gtest.h>

#include <range/v3/iterator/operations.hpp>

#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation/compute/operations/motion/se2_kinematic_state_connector.hpp>
#include <beluga_estimation/constraints/normal_constraint.hpp>
#include <beluga_estimation/constraints/se2_pose_kinematic_constraint.hpp>
#include <beluga_estimation/constraints/se2_velocity_kinematic_constraint.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace {

using namespace beluga_estimation;

/// Name of the kinematic process constraints added by the connector.
constexpr std::string_view kKinematicConstraintName = "pose/kinematics";
/// Name of the velocity random-walk constraints added by the connector.
constexpr std::string_view kNormalConstraintName = "velocity/random_walk";

class SE2KinematicStateConnectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    middleware_ = std::make_shared<beluga_estimation::MockMiddleware>();
    EXPECT_CALL(
        *(middleware_->parameters), DeclareParameter("pose_variable_name", testing::An<const ParameterValue&>()))
        .WillOnce(testing::Return(beluga_estimation::ParameterValue{"pose"}));
    EXPECT_CALL(
        *(middleware_->parameters), DeclareParameter("velocity_variable_name", testing::An<const ParameterValue&>()))
        .WillOnce(testing::Return(beluga_estimation::ParameterValue{"velocity"}));
    EXPECT_CALL(*(middleware_->parameters), DeclareParameter("pose_covariance", testing::An<const ParameterValue&>()))
        .WillOnce(testing::Return(
            beluga_estimation::ParameterValue{std::vector<double>{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01}}));
    EXPECT_CALL(
        *(middleware_->parameters), DeclareParameter("velocity_covariance", testing::An<const ParameterValue&>()))
        .WillOnce(testing::Return(
            beluga_estimation::ParameterValue{std::vector<double>{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01}}));

    connector_ = std::make_unique<beluga_estimation::SE2KinematicStateConnector>(middleware_);

    ON_CALL(*middleware_->logger, Log(testing::_, testing::_)).WillByDefault(testing::Return());
    EXPECT_CALL(*middleware_->logger, Log(testing::_, testing::_)).Times(testing::AnyNumber());
  }

  Context context_;
  std::shared_ptr<beluga_estimation::MockMiddleware> middleware_;
  std::unique_ptr<beluga_estimation::SE2KinematicStateConnector> connector_;

  TimePoint MakeTime(int seconds) { return TimePoint{std::chrono::seconds{seconds}}; }

  void AddPoseVariable(TimePoint t) {
    const Identifier id{"pose", t};
    ASSERT_TRUE(context_.dbn.Add<SE2Variable>(id));
  }

  void AddVelocityVariable(TimePoint t) { AddVelocityVariable(t, NDim<3>{}); }

  template <std::size_t N>
  void AddVelocityVariable(TimePoint t, NDim<N> ndim) {
    const Identifier id{"velocity", t};
    ASSERT_TRUE(context_.dbn.Add<EuclideanVariable>(id, ndim));
  }

  std::size_t CountConstraints(std::string_view name) {
    return static_cast<std::size_t>(ranges::distance(context_.dbn.GetConstraints(name)));
  }

  std::size_t CountVariables(std::string_view name) {
    return static_cast<std::size_t>(ranges::distance(context_.dbn.GetVariables(name)));
  }
};

TEST_F(SE2KinematicStateConnectorTest, EmptyDbnIsNoOp) {
  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_TRUE(status);

  EXPECT_EQ(CountVariables("pose"), 0u);
  EXPECT_EQ(CountVariables("velocity"), 0u);
  EXPECT_EQ(ranges::distance(context_.dbn.GetConstraints()), 0);
}

TEST_F(SE2KinematicStateConnectorTest, FillsMissingPose) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);
  AddVelocityVariable(t1);

  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_TRUE(status);

  auto pose_t1_result = context_.dbn.GetVariable<SE2Variable>({"pose", t1});
  ASSERT_TRUE(pose_t1_result);
  auto pose_t1 = Unwrap(pose_t1_result);
  ASSERT_NE(pose_t1, nullptr);
  EXPECT_EQ(pose_t1->Id().timestamp, t1);

  // The pair (t0, t1) is connected by both a kinematic and a velocity constraint.
  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 1u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 1u);
}

TEST_F(SE2KinematicStateConnectorTest, FillsMissingVelocity) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddPoseVariable(t1);

  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_TRUE(status);

  for (const auto t : {t0, t1}) {
    auto velocity_result = context_.dbn.GetVariable<EuclideanVariable>({"velocity", t});
    ASSERT_TRUE(velocity_result);
    auto velocity = Unwrap(velocity_result);
    ASSERT_NE(velocity, nullptr);
    EXPECT_EQ(velocity->Id().timestamp, t);
    EXPECT_EQ(velocity->Size(), 3u);
  }

  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 1u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 1u);
}

TEST_F(SE2KinematicStateConnectorTest, AddsConstraintsWhenAllVariablesPresent) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);
  AddPoseVariable(t1);
  AddVelocityVariable(t1);

  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_TRUE(status);

  // No variable had to be created, but the pair still lacks process models.
  EXPECT_EQ(CountVariables("pose"), 2u);
  EXPECT_EQ(CountVariables("velocity"), 2u);
  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 1u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 1u);
}

TEST_F(SE2KinematicStateConnectorTest, InterleavedGapsFilledSuccessfully) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);
  const auto t2 = MakeTime(2);
  const auto t3 = MakeTime(3);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);

  // t1 missing velocity.
  AddPoseVariable(t1);

  // t2 missing pose.
  AddVelocityVariable(t2);

  AddPoseVariable(t3);
  AddVelocityVariable(t3);

  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_TRUE(status);

  auto velocity_t1_result = context_.dbn.GetVariable<EuclideanVariable>({"velocity", t1});
  ASSERT_TRUE(velocity_t1_result);
  auto velocity_t1 = Unwrap(velocity_t1_result);
  ASSERT_NE(velocity_t1, nullptr);
  EXPECT_EQ(velocity_t1->Size(), 3u);

  auto pose_t2_result = context_.dbn.GetVariable<SE2Variable>({"pose", t2});
  ASSERT_TRUE(pose_t2_result);
  EXPECT_NE(Unwrap(pose_t2_result), nullptr);

  // Three consecutive pairs, each with both process models.
  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 3u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 3u);
}

TEST_F(SE2KinematicStateConnectorTest, HandlesUnpairedIntermediateTimestamps) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);
  const auto t3 = MakeTime(3);

  // Pose and velocity timelines do not coincide at t1: there is a velocity
  // without a pose, and t0 has a pose without a velocity.
  AddPoseVariable(t0);
  AddVelocityVariable(t1);
  AddPoseVariable(t3);
  AddVelocityVariable(t3);

  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_TRUE(status);

  EXPECT_EQ(CountVariables("pose"), 3u);
  EXPECT_EQ(CountVariables("velocity"), 3u);
  // Timeline is [t0, t1, t3]: two consecutive pairs, each with both factors.
  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 2u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 2u);
}

TEST_F(SE2KinematicStateConnectorTest, RejectsVelocityOfWrongDimension) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);
  AddPoseVariable(t1);
  AddVelocityVariable(t1, NDim<2>{});

  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kFailure);
}

TEST_F(SE2KinematicStateConnectorTest, CustomConfigurationParameters) {
  SE2KinematicStateConnector::Configuration config;
  config.pose_variable_name = "custom_pose";
  config.velocity_variable_name = "custom_vel";
  config.pose_covariance = Matrix3d::Identity() * 0.05;
  config.velocity_covariance = Matrix3d::Identity() * 0.05;

  auto custom_connector = std::make_unique<SE2KinematicStateConnector>(middleware_, config);

  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  context_.dbn.Add<SE2Variable>(Identifier{"custom_pose", t0});
  context_.dbn.Add<EuclideanVariable>(Identifier{"custom_vel", t0}, NDim<3>{});
  context_.dbn.Add<SE2Variable>(Identifier{"custom_pose", t1});

  const auto status = custom_connector->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_TRUE(status);

  auto velocity_t1_result = context_.dbn.GetVariable<EuclideanVariable>({"custom_vel", t1});
  ASSERT_TRUE(velocity_t1_result);
  auto velocity_t1 = Unwrap(velocity_t1_result);
  ASSERT_NE(velocity_t1, nullptr);
  EXPECT_EQ(velocity_t1->Id().name, "custom_vel");
  EXPECT_EQ(velocity_t1->Size(), 3u);

  // Constraint names derive from the configured variable names, not the defaults.
  EXPECT_NE(context_.dbn.GetLatestConstraint("custom_pose/kinematics"), nullptr);
  EXPECT_NE(context_.dbn.GetLatestConstraint("custom_vel/random_walk"), nullptr);
  EXPECT_EQ(context_.dbn.GetLatestConstraint("pose/kinematics"), nullptr);
  EXPECT_EQ(context_.dbn.GetLatestConstraint("velocity/random_walk"), nullptr);
}

TEST_F(SE2KinematicStateConnectorTest, MultiAgentConnectorsDoNotCollide) {
  SE2KinematicStateConnector::Configuration agent0_config;
  agent0_config.pose_variable_name = "agent0/pose";
  agent0_config.velocity_variable_name = "agent0/velocity";
  SE2KinematicStateConnector::Configuration agent1_config;
  agent1_config.pose_variable_name = "agent1/pose";
  agent1_config.velocity_variable_name = "agent1/velocity";

  auto agent0_connector = std::make_unique<SE2KinematicStateConnector>(middleware_, agent0_config);
  auto agent1_connector = std::make_unique<SE2KinematicStateConnector>(middleware_, agent1_config);

  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  // Both agents share one DBN and one timeline.
  ASSERT_TRUE(context_.dbn.Add<SE2Variable>(Identifier{"agent0/pose", t0}));
  ASSERT_TRUE(context_.dbn.Add<SE2Variable>(Identifier{"agent0/pose", t1}));
  ASSERT_TRUE(context_.dbn.Add<SE2Variable>(Identifier{"agent1/pose", t0}));
  ASSERT_TRUE(context_.dbn.Add<SE2Variable>(Identifier{"agent1/pose", t1}));

  EXPECT_TRUE(agent0_connector->Execute(context_, ComputeBudget::Unlimited()));
  EXPECT_TRUE(agent1_connector->Execute(context_, ComputeBudget::Unlimited()));

  // Every constraint of every agent must be present: a name clash would have made
  // the second agent's insertions fail and be reported as conflicts.
  EXPECT_NE(context_.dbn.GetLatestConstraint("agent0/pose/kinematics"), nullptr);
  EXPECT_NE(context_.dbn.GetLatestConstraint("agent0/velocity/random_walk"), nullptr);
  EXPECT_NE(context_.dbn.GetLatestConstraint("agent1/pose/kinematics"), nullptr);
  EXPECT_NE(context_.dbn.GetLatestConstraint("agent1/velocity/random_walk"), nullptr);
}

TEST_F(SE2KinematicStateConnectorTest, EmitsDebugSummaryWhenFillingGaps) {
  std::vector<std::string> debug_messages;
  ON_CALL(*middleware_->logger, Log(LogLevel::kDebug, testing::_))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });

  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);
  AddVelocityVariable(t1);

  ASSERT_TRUE(connector_->Execute(context_, ComputeBudget::Unlimited()));

  EXPECT_THAT(
      debug_messages,
      testing::Contains("SE2KinematicStateConnector: added 0 velocity variable(s) and 1 pose variable(s)"));
  EXPECT_THAT(
      debug_messages,
      testing::Contains("SE2KinematicStateConnector: added 1 kinematic constraint(s) and 1 normal constraint(s)"));

  EXPECT_THAT(debug_messages, testing::Contains("SE2KinematicStateConnector: processing the full network timeline"));
  EXPECT_THAT(
      debug_messages,
      testing::Contains("SE2KinematicStateConnector: pass 1 found 1 pose variable(s) and 2 velocity variable(s)"));
  EXPECT_THAT(
      debug_messages,
      testing::Contains("SE2KinematicStateConnector: pass 2 found 1 pose variable(s) and 2 velocity variable(s)"));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("added missing 'pose' variable at timestamp")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("connected pose at timestamp")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("connected velocity at timestamp")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("timeline completed up to timestamp")));
}

TEST_F(SE2KinematicStateConnectorTest, SkipsKinematicConstraintWhenNextPoseIsAlreadyConditioned) {
  std::vector<std::string> debug_messages;
  ON_CALL(*middleware_->logger, Log(LogLevel::kDebug, testing::_))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });

  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);
  AddPoseVariable(t1);
  AddVelocityVariable(t1);

  // Simulate a pre-existing process model (e.g. from the odometry connector)
  // that already conditions the second pose on the first one.
  ASSERT_TRUE(context_.dbn.Add<SE2PoseKinematicConstraint>(
      Identifier{"odom/kinematic", t1}, Identifier{"pose", t0}, Identifier{"pose", t1}, Matrix3d::Identity() * 0.01));

  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_TRUE(status);

  // The pose already has a process model, the velocity does not.
  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 0u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 1u);

  EXPECT_THAT(
      debug_messages, testing::Contains(testing::HasSubstr("skipping kinematic constraint for pose at timestamp")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("connected velocity at timestamp")));
}

TEST_F(SE2KinematicStateConnectorTest, ConnectsBoundaryPairAcrossExecutions) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);

  // A single timestamp has no pair to connect yet.
  ASSERT_TRUE(connector_->Execute(context_, ComputeBudget::Unlimited()));
  ASSERT_EQ(CountConstraints(kKinematicConstraintName), 0u);
  ASSERT_EQ(CountConstraints(kNormalConstraintName), 0u);

  // New data arrives at a later timestamp.
  AddPoseVariable(t1);
  AddVelocityVariable(t1);

  ASSERT_TRUE(connector_->Execute(context_, ComputeBudget::Unlimited()));

  // The boundary pair (t0, t1) must be connected even though t0 was already
  // processed by the previous call.
  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 1u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 1u);
}

TEST_F(SE2KinematicStateConnectorTest, DoesNotRevisitAlreadyProcessedPairs) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);
  AddPoseVariable(t1);
  AddVelocityVariable(t1);

  ASSERT_TRUE(connector_->Execute(context_, ComputeBudget::Unlimited()));
  ASSERT_EQ(CountConstraints(kKinematicConstraintName), 1u);
  ASSERT_EQ(CountConstraints(kNormalConstraintName), 1u);

  // A second call without new data must not duplicate anything.
  ASSERT_TRUE(connector_->Execute(context_, ComputeBudget::Unlimited()));
  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 1u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 1u);

  // Removing the constraint must not make the connector revisit the old pair.
  auto constraint = context_.dbn.GetLatestConstraint(kKinematicConstraintName);
  ASSERT_NE(constraint, nullptr);
  ASSERT_TRUE(context_.dbn.Remove(constraint));

  ASSERT_TRUE(connector_->Execute(context_, ComputeBudget::Unlimited()));
  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 0u);
}

TEST_F(SE2KinematicStateConnectorTest, FinalizeResetsIncrementalState) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);
  AddPoseVariable(t1);
  AddVelocityVariable(t1);

  ASSERT_TRUE(connector_->Execute(context_, ComputeBudget::Unlimited()));
  ASSERT_EQ(CountConstraints(kKinematicConstraintName), 1u);

  auto constraint = context_.dbn.GetLatestConstraint(kKinematicConstraintName);
  ASSERT_NE(constraint, nullptr);
  ASSERT_TRUE(context_.dbn.Remove(constraint));

  // After finalization the connector starts over from the whole network.
  ASSERT_TRUE(connector_->Finalize(ComputeBudget::Unlimited()));
  ASSERT_TRUE(connector_->Execute(context_, ComputeBudget::Unlimited()));

  EXPECT_EQ(CountConstraints(kKinematicConstraintName), 1u);
  EXPECT_EQ(CountConstraints(kNormalConstraintName), 1u);
}

TEST_F(SE2KinematicStateConnectorTest, ConflictingConstraintIdentifierReturnsError) {
  const auto t0 = MakeTime(0);
  const auto t1 = MakeTime(1);

  AddPoseVariable(t0);
  AddVelocityVariable(t0);
  AddPoseVariable(t1);
  AddVelocityVariable(t1);

  // Occupy the kinematic constraint identifier at t1 with an unrelated
  // constraint so that the pose at t1 remains unconditioned while its
  // identifier is taken.
  auto conflicting_constraint = std::make_shared<const NormalConstraint_<double, 3>>(
      Identifier{std::string(kKinematicConstraintName), t1}, Identifier{"velocity", t0}, Identifier{"velocity", t1},
      std::make_shared<const NormalDistribution_<double, 3>>(Matrix3d::Identity() * 0.01));
  ASSERT_TRUE(context_.dbn.Add(conflicting_constraint));

  const auto status = connector_->Execute(context_, ComputeBudget::Unlimited());
  EXPECT_FALSE(status);
  EXPECT_EQ(status.error(), Error::kFailure);
}

TEST_F(SE2KinematicStateConnectorTest, RejectsNonPositiveDefiniteCovariance) {
  SE2KinematicStateConnector::Configuration config;
  config.pose_covariance = Matrix3d::Identity() * -1.0;

  EXPECT_THROW(std::make_unique<SE2KinematicStateConnector>(middleware_, config), std::runtime_error);
}

TEST_F(SE2KinematicStateConnectorTest, RejectsMalformedCovarianceParameter) {
  auto middleware = std::make_shared<beluga_estimation::MockMiddleware>();
  EXPECT_CALL(*(middleware->parameters), DeclareParameter("pose_variable_name", testing::An<const ParameterValue&>()))
      .WillOnce(testing::Return(beluga_estimation::ParameterValue{"pose"}));
  EXPECT_CALL(
      *(middleware->parameters), DeclareParameter("velocity_variable_name", testing::An<const ParameterValue&>()))
      .WillOnce(testing::Return(beluga_estimation::ParameterValue{"velocity"}));
  // A covariance parameter must hold a full 3x3 (9 elements, row-major).
  EXPECT_CALL(*(middleware->parameters), DeclareParameter("pose_covariance", testing::An<const ParameterValue&>()))
      .WillOnce(testing::Return(beluga_estimation::ParameterValue{std::vector<double>{1.0, 0.0}}));

  EXPECT_THROW(
      beluga_estimation::SE2KinematicStateConnector::Configuration::LoadFromParams(middleware), std::runtime_error);
}

}  // namespace
