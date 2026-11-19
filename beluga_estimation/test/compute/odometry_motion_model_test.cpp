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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <range/v3/iterator/operations.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation/compute/context.hpp>
#include <beluga_estimation/compute/operations/motion/odometry.hpp>
#include <beluga_estimation/mocks/mock_clock.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/mocks/mock_odometry.hpp>
#include <beluga_estimation/mocks/mock_subscriber.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

using namespace beluga_estimation;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Invoke;
using ::testing::Return;

// =============================================================================
// ToSE2Covariance
// =============================================================================

TEST(OdometryUtilTest, ToSE2Covariance_Diagonal) {
  beluga_estimation::Matrix6d cov = beluga_estimation::Matrix6d::Zero();
  cov(0, 0) = 1.0;  // vx
  cov(1, 1) = 2.0;  // vy
  cov(5, 5) = 3.0;  // omega_z

  const beluga_estimation::Matrix3d result = beluga_estimation::ToSE2Covariance(cov);

  EXPECT_DOUBLE_EQ(result(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(result(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(result(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(result(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(result(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(result(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(result(2, 1), 0.0);
}

TEST(OdometryUtilTest, ToSE2Covariance_CrossTermsXY) {
  beluga_estimation::Matrix6d cov = beluga_estimation::Matrix6d::Zero();
  cov(0, 1) = 0.5;
  cov(1, 0) = 0.5;

  const beluga_estimation::Matrix3d result = beluga_estimation::ToSE2Covariance(cov);

  EXPECT_DOUBLE_EQ(result(0, 1), 0.5);
  EXPECT_DOUBLE_EQ(result(1, 0), 0.5);
  EXPECT_DOUBLE_EQ(result(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(result(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(result(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(result(2, 1), 0.0);
}

TEST(OdometryUtilTest, ToSE2Covariance_CrossTermsTranslationYaw) {
  beluga_estimation::Matrix6d cov = beluga_estimation::Matrix6d::Zero();
  cov(0, 5) = 0.25;  // x-yaw
  cov(1, 5) = 0.5;   // y-yaw
  cov(5, 0) = 0.25;
  cov(5, 1) = 0.5;

  const beluga_estimation::Matrix3d result = beluga_estimation::ToSE2Covariance(cov);

  EXPECT_DOUBLE_EQ(result(0, 2), 0.25);
  EXPECT_DOUBLE_EQ(result(1, 2), 0.5);
  EXPECT_DOUBLE_EQ(result(2, 0), 0.25);
  EXPECT_DOUBLE_EQ(result(2, 1), 0.5);
}

// =============================================================================
// Plugin integration tests — fixture
// =============================================================================

class OdometryPluginTest : public ::testing::Test {
 protected:
  static constexpr double kTol = 1e-6;

  void SetUp() override {
    middleware_ = std::make_shared<MockMiddleware>();
    clock_ = std::make_shared<MockClock>();

    ON_CALL(*middleware_, GetClockInterface()).WillByDefault(Return(clock_.get()));
    EXPECT_CALL(*middleware_, GetClockInterface()).Times(AnyNumber());

    ON_CALL(*middleware_->logger, Log(_, _)).WillByDefault(Return());
    EXPECT_CALL(*middleware_->logger, Log(_, _)).Times(AnyNumber());

    ON_CALL(*middleware_->topics, CreateSubscriberImpl(_, _, _))
        .WillByDefault(Invoke([this](MessageId, std::string_view, SubscriberCallback<Message> cb) {
          odometry_callback_ = std::move(cb);
          return std::make_shared<MockSubscriber<beluga_estimation::msg::Odometry>>();
        }));
    EXPECT_CALL(*middleware_->topics, CreateSubscriberImpl(_, _, _)).Times(AnyNumber());

    SE2OdometryConnector_<double>::Configuration config;
    config.device_name = "odom";
    config.topic_name = "odom";
    config.variable_name = "pose";
    config.max_buffer_age = std::chrono::seconds(5);

    plugin_ = std::make_unique<SE2OdometryConnector_<double>>(middleware_, config);
  }

  void Initialize() {
    ASSERT_TRUE(plugin_->Initialize(budget_));
    ASSERT_TRUE(odometry_callback_) << "Subscribe callback was not captured";
  }

  void InjectOdometry(
      TimePoint t,
      const beluga_estimation::SE2d& pose,
      const beluga_estimation::Matrix3d& lin_vel_cov = beluga_estimation::Matrix3d::Identity() * 0.01,
      const beluga_estimation::Matrix3d& ang_vel_cov = beluga_estimation::Matrix3d::Identity() * 0.01) {
    auto msg = std::make_shared<MockOdometry>(t, pose, lin_vel_cov, ang_vel_cov);
    odometry_callback_(msg);
  }

  std::shared_ptr<SE2Variable_<double>> MakePoseVariable(TimePoint t) {
    return std::make_shared<SE2Variable_<double>>(Identifier{"pose", t});
  }

  // Populates the plugin buffer by injecting messages and draining via Execute on an empty DBN.
  void PopulateBuffer(const std::vector<std::pair<TimePoint, beluga_estimation::SE2d>>& entries) {
    for (const auto& [t, pose] : entries) {
      InjectOdometry(t, pose);
    }
    Context empty_ctx;
    plugin_->Execute(empty_ctx, budget_);  // drains queue, returns BUSY — we don't care about status here
  }

  static TimePoint Secs(int s) { return TimePoint(std::chrono::seconds(s)); }

  std::shared_ptr<MockMiddleware> middleware_;
  std::shared_ptr<MockClock> clock_;
  std::unique_ptr<SE2OdometryConnector_<double>> plugin_;
  SubscriberCallback<Message> odometry_callback_;
  ComputeBudget budget_ = ComputeBudget::Unlimited();
};

// =============================================================================
// Initialize / Finalize
// =============================================================================

TEST_F(OdometryPluginTest, Initialize_CapturesSubscriberCallback) {
  EXPECT_FALSE(odometry_callback_);
  ASSERT_TRUE(plugin_->Initialize(budget_));
  EXPECT_TRUE(odometry_callback_);
}

TEST_F(OdometryPluginTest, Finalize_ClearsBuffer) {
  Initialize();
  InjectOdometry(Secs(1), beluga_estimation::SE2d{});
  InjectOdometry(Secs(2), beluga_estimation::SE2d{});

  plugin_->Finalize(budget_);
  plugin_->Initialize(budget_);

  // After Finalize+re-Initialize, buffer is empty → Execute returns BUSY
  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  ctx.dbn.Add(MakePoseVariable(Secs(2)));
  EXPECT_EQ(plugin_->Execute(ctx, budget_).error(), Error::kBusy);
}

// =============================================================================
// LookupAt
// =============================================================================

TEST_F(OdometryPluginTest, LookupAt_EmptyBufferReturnsNullopt) {
  Initialize();
  EXPECT_FALSE(plugin_->LookupAt(Secs(0)).has_value());
}

TEST_F(OdometryPluginTest, LookupAt_ExactTimestampMatch) {
  Initialize();
  const beluga_estimation::SE2d pose(beluga_estimation::SO2d{0.5}, beluga_estimation::Vector2d{3.0, 4.0});
  PopulateBuffer({{Secs(1), pose}});

  const auto entry = plugin_->LookupAt(Secs(1));
  ASSERT_TRUE(entry.has_value());
  EXPECT_NEAR(entry->pose.translation().x(), 3.0, kTol);
  EXPECT_NEAR(entry->pose.translation().y(), 4.0, kTol);
  EXPECT_NEAR(entry->pose.so2().log(), 0.5, kTol);
}

TEST_F(OdometryPluginTest, LookupAt_BeforeFirstEntryReturnsNullopt) {
  Initialize();
  PopulateBuffer({{Secs(2), beluga_estimation::SE2d{}}});
  EXPECT_FALSE(plugin_->LookupAt(Secs(1)).has_value());
}

TEST_F(OdometryPluginTest, LookupAt_AfterLastEntryReturnsNullopt) {
  Initialize();
  PopulateBuffer({{Secs(1), beluga_estimation::SE2d{}}});
  EXPECT_FALSE(plugin_->LookupAt(Secs(2)).has_value());
}

TEST_F(OdometryPluginTest, LookupAt_InterpolatesAtMidpoint) {
  Initialize();
  const beluga_estimation::SE2d pose_a(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{0.0, 0.0});
  const beluga_estimation::SE2d pose_b(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{4.0, 0.0});
  PopulateBuffer({{Secs(0), pose_a}, {Secs(2), pose_b}});

  const auto entry = plugin_->LookupAt(Secs(1));
  ASSERT_TRUE(entry.has_value());
  EXPECT_NEAR(entry->pose.translation().x(), 2.0, kTol);
  EXPECT_NEAR(entry->pose.translation().y(), 0.0, kTol);
}

TEST_F(OdometryPluginTest, LookupAt_InterpolationPreservesVelocityCovariance) {
  Initialize();
  // Both lin and ang covariances set to scalar * I so ToSE2Covariance
  // produces a strictly positive definite diag(s, s, s) — required for Log-Euclidean logm.
  const beluga_estimation::Matrix3d lin_cov_a = beluga_estimation::Matrix3d::Identity() * 1.0;
  const beluga_estimation::Matrix3d ang_cov_a = beluga_estimation::Matrix3d::Identity() * 1.0;
  const beluga_estimation::Matrix3d lin_cov_b = beluga_estimation::Matrix3d::Identity() * 4.0;
  const beluga_estimation::Matrix3d ang_cov_b = beluga_estimation::Matrix3d::Identity() * 4.0;

  InjectOdometry(Secs(0), beluga_estimation::SE2d{}, lin_cov_a, ang_cov_a);
  InjectOdometry(Secs(2), beluga_estimation::SE2d{}, lin_cov_b, ang_cov_b);
  Context empty_ctx;
  plugin_->Execute(empty_ctx, budget_);  // drain

  const auto entry = plugin_->LookupAt(Secs(1));
  ASSERT_TRUE(entry.has_value());
  // Log-Euclidean interpolation at alpha=0.5: geometric mean sqrt(1.0 * 4.0) = 2.0.
  // Linear interpolation would give (1.0 + 4.0) / 2 = 2.5, inflating uncertainty.
  EXPECT_NEAR(entry->velocity_covariance(0, 0), 2.0, kTol);
}

// =============================================================================
// Execute — buffer and DBN interactions
// =============================================================================

TEST_F(OdometryPluginTest, Execute_NoMessagesReturnsBusy) {
  Initialize();
  Context ctx;
  EXPECT_EQ(plugin_->Execute(ctx, budget_).error(), Error::kBusy);
}

TEST_F(OdometryPluginTest, Execute_NoDBNVariablesReturnsBusy) {
  Initialize();
  InjectOdometry(Secs(1), beluga_estimation::SE2d{});
  Context ctx;
  EXPECT_EQ(plugin_->Execute(ctx, budget_).error(), Error::kBusy);
}

TEST_F(OdometryPluginTest, Execute_SingleVariableNoPairReturnsBusy) {
  Initialize();
  InjectOdometry(Secs(1), beluga_estimation::SE2d{});
  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  EXPECT_EQ(plugin_->Execute(ctx, budget_).error(), Error::kBusy);
}

TEST_F(OdometryPluginTest, Execute_MissingBufferDataReturnsBusy) {
  Initialize();
  // Only has data for t=5; variables are at t=1, t=2 — outside buffer range
  InjectOdometry(Secs(5), beluga_estimation::SE2d{});
  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  ctx.dbn.Add(MakePoseVariable(Secs(2)));
  EXPECT_EQ(plugin_->Execute(ctx, budget_).error(), Error::kBusy);
}

TEST_F(OdometryPluginTest, Execute_TwoPosesAddsConstraintAndReturnsOk) {
  Initialize();
  const beluga_estimation::SE2d pose_a(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{0.0, 0.0});
  const beluga_estimation::SE2d pose_b(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 0.0});
  InjectOdometry(Secs(1), pose_a);
  InjectOdometry(Secs(2), pose_b);

  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  ctx.dbn.Add(MakePoseVariable(Secs(2)));

  EXPECT_TRUE(plugin_->Execute(ctx, budget_));

  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints(TimeInterval{})), 1);
}

TEST_F(OdometryPluginTest, Execute_ThreeVariablesAddsTwoConstraints) {
  Initialize();
  InjectOdometry(Secs(1), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{0.0, 0.0}));
  InjectOdometry(Secs(2), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 0.0}));
  InjectOdometry(Secs(3), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{2.0, 0.0}));

  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  ctx.dbn.Add(MakePoseVariable(Secs(2)));
  ctx.dbn.Add(MakePoseVariable(Secs(3)));

  EXPECT_TRUE(plugin_->Execute(ctx, budget_));

  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints(TimeInterval{})), 2);
}

TEST_F(OdometryPluginTest, Execute_EmitsDebugSummaryForConnectedPairs) {
  Initialize();
  InjectOdometry(Secs(1), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{0.0, 0.0}));
  InjectOdometry(Secs(2), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 0.0}));

  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  ctx.dbn.Add(MakePoseVariable(Secs(2)));

  std::vector<std::string> debug_messages;
  ON_CALL(*middleware_->logger, Log(LogLevel::kDebug, _))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });

  ASSERT_TRUE(plugin_->Execute(ctx, budget_));

  EXPECT_THAT(debug_messages, ::testing::Contains("SE2OdometryConnector: connected 1 pose pair(s) from device 'odom'"));
}

TEST_F(OdometryPluginTest, Execute_AdvancesLastConnectedTimestamp) {
  Initialize();
  InjectOdometry(Secs(1), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{0.0, 0.0}));
  InjectOdometry(Secs(2), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 0.0}));

  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  ctx.dbn.Add(MakePoseVariable(Secs(2)));

  ASSERT_TRUE(plugin_->Execute(ctx, budget_));

  // Now add a third variable and a new odometry entry; Execute should connect only the new pair
  InjectOdometry(Secs(3), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{2.0, 0.0}));
  ctx.dbn.Add(MakePoseVariable(Secs(3)));

  EXPECT_TRUE(plugin_->Execute(ctx, budget_));

  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints(TimeInterval{})), 2);
}

TEST_F(OdometryPluginTest, Execute_DeltaEncodesRelativeTransform) {
  Initialize();
  const beluga_estimation::SE2d pose_a(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 0.0});
  const beluga_estimation::SE2d pose_b(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{3.0, 0.0});
  InjectOdometry(Secs(1), pose_a);
  InjectOdometry(Secs(2), pose_b);

  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  ctx.dbn.Add(MakePoseVariable(Secs(2)));

  ASSERT_TRUE(plugin_->Execute(ctx, budget_));

  // Verify delta: CalcMean(pose_a) should return pose_b (= pose_a * delta).
  for (auto&& c : ctx.dbn.template GetConstraints<SE2PoseKinematicConstraint_<double>>(TimeInterval{})) {
    ASSERT_TRUE(c);
    auto constraint = Unwrap(c);
    ASSERT_NE(constraint, nullptr);
    beluga_estimation::SE2d mean_out;
    constraint->CalcMean(pose_a, beluga_estimation::Map<beluga_estimation::SE2d>(mean_out.data()));
    EXPECT_NEAR(mean_out.translation().x(), pose_b.translation().x(), kTol);
    EXPECT_NEAR(mean_out.translation().y(), pose_b.translation().y(), kTol);
  }
}

TEST_F(OdometryPluginTest, Execute_ConstraintsRetrievableByName) {
  Initialize();
  InjectOdometry(Secs(1), beluga_estimation::SE2d{});
  InjectOdometry(Secs(2), beluga_estimation::SE2d{});
  InjectOdometry(Secs(3), beluga_estimation::SE2d{});

  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(1)));
  ctx.dbn.Add(MakePoseVariable(Secs(2)));
  ctx.dbn.Add(MakePoseVariable(Secs(3)));

  ASSERT_TRUE(plugin_->Execute(ctx, budget_));

  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints("pose/odometry")), 2);
}

TEST_F(OdometryPluginTest, MultiAgentConnectorsDoNotCollide) {
  SE2OdometryConnector_<double>::Configuration config;
  // Both agents keep the default device name; only the variable names differ.
  config.device_name = "odom";
  config.topic_name = "odom";
  config.max_buffer_age = std::chrono::seconds(5);

  config.variable_name = "agent0/pose";
  auto agent0 = std::make_unique<SE2OdometryConnector_<double>>(middleware_, config);
  config.variable_name = "agent1/pose";
  auto agent1 = std::make_unique<SE2OdometryConnector_<double>>(middleware_, config);

  Context ctx;
  ASSERT_TRUE(ctx.dbn.Add(std::make_shared<SE2Variable_<double>>(Identifier{"agent0/pose", Secs(1)})));
  ASSERT_TRUE(ctx.dbn.Add(std::make_shared<SE2Variable_<double>>(Identifier{"agent0/pose", Secs(2)})));
  ASSERT_TRUE(ctx.dbn.Add(std::make_shared<SE2Variable_<double>>(Identifier{"agent1/pose", Secs(1)})));
  ASSERT_TRUE(ctx.dbn.Add(std::make_shared<SE2Variable_<double>>(Identifier{"agent1/pose", Secs(2)})));

  ASSERT_TRUE(agent0->Initialize(budget_));
  InjectOdometry(Secs(1), beluga_estimation::SE2d{});
  InjectOdometry(Secs(2), beluga_estimation::SE2d{});
  ASSERT_TRUE(agent0->Execute(ctx, budget_));

  ASSERT_TRUE(agent1->Initialize(budget_));
  InjectOdometry(Secs(1), beluga_estimation::SE2d{});
  InjectOdometry(Secs(2), beluga_estimation::SE2d{});
  ASSERT_TRUE(agent1->Execute(ctx, budget_));

  // A name clash would have made the second agent's insertions fail and be skipped.
  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints("agent0/pose/odometry")), 1);
  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints("agent1/pose/odometry")), 1);
}

// =============================================================================
// DrainQueue
// =============================================================================

TEST_F(OdometryPluginTest, DrainQueue_DuplicateTimestampKeepsLatest) {
  Initialize();
  const beluga_estimation::SE2d first(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 0.0});
  const beluga_estimation::SE2d second(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{99.0, 0.0});

  InjectOdometry(Secs(1), first);
  InjectOdometry(Secs(1), second);  // same timestamp — should overwrite

  Context empty_ctx;
  plugin_->Execute(empty_ctx, budget_);  // drain

  const auto entry = plugin_->LookupAt(Secs(1));
  ASSERT_TRUE(entry.has_value());
  EXPECT_NEAR(entry->pose.translation().x(), 99.0, kTol);
}

// =============================================================================
// EvictBuffer
// =============================================================================

TEST_F(OdometryPluginTest, Eviction_NoAnchorRemovesEntriesOlderThanMaxAge) {
  Initialize();
  // Inject entries at t=0, t=1, t=2 (all within 5s age initially)
  InjectOdometry(Secs(0), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{0.0, 0.0}));
  InjectOdometry(Secs(1), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 0.0}));
  InjectOdometry(Secs(2), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{2.0, 0.0}));

  // Advance clock to t=8: max_age_cutoff = 8 - 5 = 3. Entries at 0,1,2 are all older than 3.
  clock_->SetTime(Secs(8));

  Context empty_ctx;
  plugin_->Execute(empty_ctx, budget_);  // drain + evict

  EXPECT_FALSE(plugin_->LookupAt(Secs(0)).has_value());
  EXPECT_FALSE(plugin_->LookupAt(Secs(1)).has_value());
  EXPECT_FALSE(plugin_->LookupAt(Secs(2)).has_value());
}

TEST_F(OdometryPluginTest, Eviction_NoAnchorKeepsRecentEntries) {
  Initialize();
  InjectOdometry(Secs(0), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{0.0, 0.0}));
  InjectOdometry(Secs(6), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{6.0, 0.0}));

  // Advance clock to t=8: max_age_cutoff = 3. Entry at t=6 is kept; t=0 is evicted.
  clock_->SetTime(Secs(8));

  Context empty_ctx;
  plugin_->Execute(empty_ctx, budget_);

  EXPECT_FALSE(plugin_->LookupAt(Secs(0)).has_value());
  EXPECT_TRUE(plugin_->LookupAt(Secs(6)).has_value());
}

TEST_F(OdometryPluginTest, Eviction_WithAnchorPreservesEntryBeforeAnchor) {
  Initialize();
  // Inject at t=1, t=2, t=3 and connect pair (t=2, t=3) → anchor = t=3
  InjectOdometry(Secs(1), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{1.0, 0.0}));
  InjectOdometry(Secs(2), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{2.0, 0.0}));
  InjectOdometry(Secs(3), beluga_estimation::SE2d(beluga_estimation::SO2d{0.0}, beluga_estimation::Vector2d{3.0, 0.0}));

  Context ctx;
  ctx.dbn.Add(MakePoseVariable(Secs(2)));
  ctx.dbn.Add(MakePoseVariable(Secs(3)));
  ASSERT_TRUE(plugin_->Execute(ctx, budget_));

  // EvictBuffer runs at the start of Execute, before the anchor is set.
  // A second Execute is needed to trigger eviction with anchor=t=3.
  plugin_->Execute(ctx, budget_);

  // Anchor=t=3: one entry before anchor (t=2) is retained; t=1 is erased.
  EXPECT_FALSE(plugin_->LookupAt(Secs(1)).has_value());
  EXPECT_TRUE(plugin_->LookupAt(Secs(2)).has_value());
  EXPECT_TRUE(plugin_->LookupAt(Secs(3)).has_value());
}
