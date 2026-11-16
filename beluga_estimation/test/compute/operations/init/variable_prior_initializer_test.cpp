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

#include <Eigen/Core>
#include <range/v3/iterator/operations.hpp>
#include <range/v3/range/conversion.hpp>

#include <beluga_estimation/common/registry.hpp>
#include <beluga_estimation/compute/context.hpp>
#include <beluga_estimation/compute/operations/variable_prior_initializer.hpp>
#include <beluga_estimation/constraints/normal_prior_constraint.hpp>
#include <beluga_estimation/constraints/se2_normal_pose_prior_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/mocks/mock_clock.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace {

using namespace beluga_estimation;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Return;

using Configuration = VariablePriorInitializer::Configuration;
using VariableSpec = Configuration::VariableSpec;

class VariablePriorInitializerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    middleware_ = std::make_shared<MockMiddleware>();
    clock_ = std::make_shared<MockClock>();

    ON_CALL(*middleware_, GetClockInterface()).WillByDefault(Return(clock_.get()));
    EXPECT_CALL(*middleware_, GetClockInterface()).Times(AnyNumber());

    ON_CALL(*middleware_->logger, Log(_, _)).WillByDefault(Return());
    EXPECT_CALL(*middleware_->logger, Log(_, _)).Times(AnyNumber());
  }

  // A valid configuration with one SE(2) variable and one 2D Euclidean variable.
  static Configuration MakeMixedConfig() {
    Configuration config;
    config.specs = {
        {"pose", "se2", {1.0, 2.0, 0.5}, {0.1, 0.05, 0.0, 0.05, 0.2, 0.0, 0.0, 0.0, 0.3}},
        {"bias", "euclidean", {0.5, -0.5}, {0.4, 0.1, 0.1, 0.6}},
    };
    return config;
  }

  std::shared_ptr<MockMiddleware> middleware_;
  std::shared_ptr<MockClock> clock_;
  ComputeBudget budget_{ComputeBudget::Unlimited()};
};

// =============================================================================
// Construction
// =============================================================================

TEST_F(VariablePriorInitializerTest, ConstructionWithExplicitConfig) {
  EXPECT_NO_THROW({ VariablePriorInitializer initializer(middleware_, MakeMixedConfig()); });
}

TEST_F(VariablePriorInitializerTest, ConstructionFromParams) {
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.variables", ParameterType::kStringArray))
      .WillOnce(Return(ParameterValue{std::vector<std::string>{"pose", "bias"}}));

  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.pose.type", ParameterType::kString))
      .WillOnce(Return(ParameterValue{std::string("se2")}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.pose.mean", ParameterType::kDoubleArray))
      .WillOnce(Return(ParameterValue{std::vector<double>{1.0, 2.0, 0.5}}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.pose.covariance", ParameterType::kDoubleArray))
      .WillOnce(Return(ParameterValue{std::vector<double>{0.1, 0.05, 0.0, 0.05, 0.2, 0.0, 0.0, 0.0, 0.3}}));

  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.bias.type", ParameterType::kString))
      .WillOnce(Return(ParameterValue{std::string("euclidean")}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.bias.mean", ParameterType::kDoubleArray))
      .WillOnce(Return(ParameterValue{std::vector<double>{0.5, -0.5}}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.bias.covariance", ParameterType::kDoubleArray))
      .WillOnce(Return(ParameterValue{std::vector<double>{0.4, 0.1, 0.1, 0.6}}));

  EXPECT_NO_THROW({ VariablePriorInitializer initializer(middleware_, "test."); });
}

// =============================================================================
// Initialization (configuration validation)
// =============================================================================

TEST_F(VariablePriorInitializerTest, InitializeSucceedsOnValidConfiguration) {
  VariablePriorInitializer initializer(middleware_, MakeMixedConfig());
  EXPECT_TRUE(initializer.Initialize(budget_));
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnUnknownType) {
  auto config = MakeMixedConfig();
  config.specs[0].type = "foo";
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnSE2MeanSizeMismatch) {
  auto config = MakeMixedConfig();
  config.specs[0].mean = {1.0, 2.0};
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnSE2CovarianceSizeMismatch) {
  auto config = MakeMixedConfig();
  config.specs[0].covariance = {0.1, 0.05, 0.0, 0.05, 0.2, 0.0, 0.0, 0.0};
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnEmptyEuclideanMean) {
  auto config = MakeMixedConfig();
  config.specs[1].mean = {};
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnEuclideanCovarianceSizeMismatch) {
  auto config = MakeMixedConfig();
  config.specs[1].covariance = {0.4, 0.1, 0.1, 0.6, 0.0};
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnNonPositiveDefiniteCovariance) {
  auto config = MakeMixedConfig();
  config.specs[0].covariance = {-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnEuclideanNonPositiveDefiniteCovariance) {
  auto config = MakeMixedConfig();
  config.specs[1].covariance = {0.4, 0.1, 0.1, -0.6};
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnAsymmetricCovariance) {
  // The lower triangle is positive definite but the matrix is not symmetric;
  // the symmetry check must reject it before any Cholesky-based use.
  auto config = MakeMixedConfig();
  config.specs[0].covariance = {0.1, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3};
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnAsymmetricEuclideanCovariance) {
  auto config = MakeMixedConfig();
  config.specs[1].covariance = {0.4, 0.5, 0.1, 0.6};
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnDuplicateVariableNames) {
  auto config = MakeMixedConfig();
  config.specs.push_back(config.specs[0]);
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnEmptyVariableName) {
  auto config = MakeMixedConfig();
  config.specs[0].name = "";
  VariablePriorInitializer initializer(middleware_, config);
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

// =============================================================================
// Execution
// =============================================================================

TEST_F(VariablePriorInitializerTest, ExecuteAddsSE2AndEuclideanVariablesAndPriors) {
  VariablePriorInitializer initializer(middleware_, MakeMixedConfig());
  Context ctx;

  ASSERT_TRUE(initializer.Initialize(budget_));
  EXPECT_TRUE(initializer.Execute(ctx, budget_));

  // One SE(2) variable and one Euclidean variable were added.
  EXPECT_EQ(ranges::distance(ctx.dbn.GetVariables()), 2);
  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints()), 2);

  auto pose_vars = ctx.dbn.GetVariables<SE2Variable>("pose") | ranges::to_vector;
  ASSERT_EQ(pose_vars.size(), 1);
  ASSERT_TRUE(pose_vars.front());
  EXPECT_EQ(Unwrap(pose_vars.front())->Id().timestamp, TimePoint{std::chrono::seconds(0)});

  auto bias_vars = ctx.dbn.GetVariables<EuclideanVariable>("bias") | ranges::to_vector;
  ASSERT_EQ(bias_vars.size(), 1);
  ASSERT_TRUE(bias_vars.front());
  EXPECT_EQ(Unwrap(bias_vars.front())->Manifold()->TangentSize(), 2);

  // The SE(2) prior carries the pose-form mean and the tangent covariance.
  auto se2_prior_result = ctx.dbn.GetLatestConstraint<SE2NormalPosePriorConstraint>("pose/prior");
  ASSERT_TRUE(se2_prior_result);
  auto se2_prior = Unwrap(se2_prior_result);
  ASSERT_NE(se2_prior, nullptr);
  beluga_estimation::SE2d se2_mean;
  se2_prior->CalcMean(se2_mean);
  EXPECT_NEAR(se2_mean.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(se2_mean.translation().y(), 2.0, 1e-6);
  EXPECT_NEAR(se2_mean.so2().log(), 0.5, 1e-6);
  beluga_estimation::Matrix3d se2_cov;
  se2_prior->CalcCovariance(se2_cov);
  EXPECT_NEAR(se2_cov(0, 0), 0.1, 1e-6);
  EXPECT_NEAR(se2_cov(0, 1), 0.05, 1e-6);
  EXPECT_NEAR(se2_cov(1, 0), 0.05, 1e-6);
  EXPECT_NEAR(se2_cov(1, 1), 0.2, 1e-6);
  EXPECT_NEAR(se2_cov(2, 2), 0.3, 1e-6);

  // The Euclidean prior carries the vector mean and the covariance.
  auto euclidean_prior_result = ctx.dbn.GetLatestConstraint<NormalPriorConstraint<>>("bias/prior");
  ASSERT_TRUE(euclidean_prior_result);
  auto euclidean_prior = Unwrap(euclidean_prior_result);
  ASSERT_NE(euclidean_prior, nullptr);
  beluga_estimation::VectorXd euclidean_mean(2);
  euclidean_prior->CalcMean(euclidean_mean);
  EXPECT_NEAR(euclidean_mean(0), 0.5, 1e-6);
  EXPECT_NEAR(euclidean_mean(1), -0.5, 1e-6);
  beluga_estimation::MatrixXd euclidean_cov(2, 2);
  euclidean_prior->CalcCovariance(euclidean_cov);
  EXPECT_NEAR(euclidean_cov(0, 0), 0.4, 1e-6);
  EXPECT_NEAR(euclidean_cov(0, 1), 0.1, 1e-6);
  EXPECT_NEAR(euclidean_cov(1, 1), 0.6, 1e-6);
}

TEST_F(VariablePriorInitializerTest, ExecuteSecondCallIsNoOp) {
  VariablePriorInitializer initializer(middleware_, MakeMixedConfig());
  Context ctx;

  ASSERT_TRUE(initializer.Initialize(budget_));
  ASSERT_TRUE(initializer.Execute(ctx, budget_));
  const auto variables_before = ranges::distance(ctx.dbn.GetVariables());
  const auto constraints_before = ranges::distance(ctx.dbn.GetConstraints());
  EXPECT_EQ(variables_before, 2);
  EXPECT_EQ(constraints_before, 2);

  EXPECT_TRUE(initializer.Execute(ctx, budget_));
  EXPECT_EQ(ranges::distance(ctx.dbn.GetVariables()), variables_before);
  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints()), constraints_before);
}

TEST_F(VariablePriorInitializerTest, InitializeFailsOnNoSpecs) {
  // A configuration with no variable specifications is a configuration error.
  VariablePriorInitializer initializer(middleware_, Configuration{});
  EXPECT_EQ(initializer.Initialize(budget_).error(), Error::kFailure);
}

TEST_F(VariablePriorInitializerTest, FinalizeAllowsReinitialization) {
  VariablePriorInitializer initializer(middleware_, MakeMixedConfig());
  Context ctx;

  ASSERT_TRUE(initializer.Initialize(budget_));
  ASSERT_TRUE(initializer.Execute(ctx, budget_));
  EXPECT_EQ(ranges::distance(ctx.dbn.GetVariables()), 2);

  EXPECT_TRUE(initializer.Finalize(budget_));
  clock_->Advance(std::chrono::seconds(1));

  EXPECT_TRUE(initializer.Execute(ctx, budget_));
  EXPECT_EQ(ranges::distance(ctx.dbn.GetVariables()), 4);
  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints()), 4);
}

TEST_F(VariablePriorInitializerTest, ExecuteFailsWhenVariableAlreadyExists) {
  // A pre-existing variable with the same name and timestamp cannot be added;
  // the call fails as soon as the collision is hit.
  Context ctx;
  const auto now = TimePoint{std::chrono::seconds(0)};
  ctx.dbn.Add(std::make_shared<SE2Variable>(Identifier{"pose", now}));

  VariablePriorInitializer initializer(middleware_, MakeMixedConfig());
  ASSERT_TRUE(initializer.Initialize(budget_));

  EXPECT_EQ(initializer.Execute(ctx, budget_).error(), Error::kFailure);
  // Only the pre-existing variable remains; nothing else was added.
  EXPECT_EQ(ranges::distance(ctx.dbn.GetVariables()), 1);
  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints()), 0);
}

TEST_F(VariablePriorInitializerTest, FloatInstantiation) {
  VariablePriorInitializer_<float>::Configuration config;
  config.specs = {{"bias", "euclidean", {0.5, -0.5}, {0.4, 0.1, 0.1, 0.6}}};

  VariablePriorInitializer_<float> initializer(middleware_, std::move(config));
  Context_<float> ctx;

  ASSERT_TRUE(initializer.Initialize(budget_));
  EXPECT_TRUE(initializer.Execute(ctx, budget_));
  EXPECT_EQ(ranges::distance(ctx.dbn.GetVariables()), 1);
  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints()), 1);
  auto bias_vars = ctx.dbn.GetVariables<EuclideanVariable_<float>>("bias") | ranges::to_vector;
  ASSERT_EQ(bias_vars.size(), 1);
  ASSERT_TRUE(bias_vars.front());
  EXPECT_EQ(Unwrap(bias_vars.front())->Manifold()->TangentSize(), 2);
}

// =============================================================================
// Registry integration
// =============================================================================

TEST_F(VariablePriorInitializerTest, RegisteredInRegistryAndInstantiableFromParams) {
  auto registry = beluga_estimation::Registry<beluga_estimation::BayesianOperation>::Instance();
  EXPECT_TRUE(registry->HasType("beluga_estimation::VariablePriorInitializer"));

  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.variables", ParameterType::kStringArray))
      .WillOnce(Return(ParameterValue{std::vector<std::string>{"pose"}}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.pose.type", ParameterType::kString))
      .WillOnce(Return(ParameterValue{std::string("se2")}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.pose.mean", ParameterType::kDoubleArray))
      .WillOnce(Return(ParameterValue{std::vector<double>{1.0, 2.0, 0.5}}));
  EXPECT_CALL(*middleware_->parameters, DeclareParameter("test.pose.covariance", ParameterType::kDoubleArray))
      .WillOnce(Return(ParameterValue{std::vector<double>{0.1, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3}}));

  auto instance = registry->MakeInstance("beluga_estimation::VariablePriorInitializer", middleware_, "test.");
  ASSERT_TRUE(instance);
  auto operation = Unwrap(instance);
  ASSERT_NE(operation, nullptr);

  Context ctx;
  EXPECT_TRUE(operation->Initialize(budget_));
  EXPECT_TRUE(operation->Execute(ctx, budget_));
  EXPECT_EQ(ranges::distance(ctx.dbn.GetVariables()), 1);
  EXPECT_EQ(ranges::distance(ctx.dbn.GetConstraints()), 1);
}

// Success: the initialization summary is reported at debug level
TEST_F(VariablePriorInitializerTest, EmitsDebugSummaryWhenInitializing) {
  std::vector<std::string> debug_messages;
  ON_CALL(*middleware_->logger, Log(LogLevel::kDebug, _))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });

  VariablePriorInitializer initializer(middleware_, MakeMixedConfig());
  Context ctx;

  ASSERT_TRUE(initializer.Initialize(budget_));
  ASSERT_TRUE(initializer.Execute(ctx, budget_));

  EXPECT_THAT(debug_messages, ::testing::Contains("VariablePriorInitializer: initialized 2 variable prior(s)"));
}

}  // namespace
