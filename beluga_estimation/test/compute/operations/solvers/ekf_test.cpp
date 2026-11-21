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

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <range/v3/range/primitives.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation/compute/context.hpp>
#include <beluga_estimation/compute/operations/solvers/ekf.hpp>
#include <beluga_estimation/constraints/normal_constraint.hpp>
#include <beluga_estimation/constraints/normal_likelihood_constraint.hpp>
#include <beluga_estimation/constraints/normal_prior_constraint.hpp>
#include <beluga_estimation/constraints/se2_constant_velocity_kinematic_constraint.hpp>
#include <beluga_estimation/constraints/se2_normal_pose_likelihood_constraint.hpp>
#include <beluga_estimation/constraints/se2_normal_pose_prior_constraint.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/distributions/se2_normal_distribution.hpp>
#include <beluga_estimation/manifolds/euclidean_manifold.hpp>
#include <beluga_estimation/mocks/mock_manifold.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>
#include <beluga_estimation/mocks/mock_variable.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

namespace {

using namespace beluga_estimation;
using ::testing::_;
using ::testing::Return;

constexpr double kTolerance = 5e-4;

constexpr std::size_t kPoseTangent = 3;
constexpr std::size_t kVelTangent = 3;

class ExtendedKalmanFilterTest : public ::testing::Test {
 protected:
  static std::shared_ptr<ExtendedKalmanFilter> MakeFilter(std::vector<std::string> names) {
    ExtendedKalmanFilter::Configuration config;
    config.tracked_variable_names = std::move(names);
    auto middleware = std::make_shared<MockMiddleware>();
    return std::make_shared<ExtendedKalmanFilter>(middleware, config);
  }

  template <int N>
  static std::shared_ptr<EuclideanVariable>
  AddEuclideanVariable(Context& context, const std::string& name, TimePoint time) {
    auto variable = std::make_shared<EuclideanVariable>(Identifier{name, time}, beluga_estimation::NDim<N>{});
    EXPECT_TRUE(context.dbn.Add(variable));
    return variable;
  }

  template <int N>
  static void AddNormalPrior(
      Context& context,
      std::string name,
      const Identifier& variable,
      const beluga_estimation::Vector<DefaultScalar, N>& mean,
      const beluga_estimation::Matrix<DefaultScalar, N, N>& covariance) {
    auto prior = std::make_shared<NormalPriorConstraint_<DefaultScalar, N>>(
        Identifier{std::move(name), variable.timestamp}, variable,
        std::make_shared<NormalDistribution_<DefaultScalar, N>>(mean, covariance));
    EXPECT_TRUE(context.dbn.Add(prior));
  }

  template <int N>
  static void AddNormalDelta(
      Context& context,
      std::string name,
      const Identifier& from,
      const Identifier& to,
      const beluga_estimation::Vector<DefaultScalar, N>& bias,
      const beluga_estimation::Matrix<DefaultScalar, N, N>& covariance) {
    auto process = std::make_shared<NormalConstraint_<DefaultScalar, N>>(
        Identifier{std::move(name), to.timestamp}, from, to,
        std::make_shared<NormalDistribution_<DefaultScalar, N>>(bias, covariance));
    EXPECT_TRUE(context.dbn.Add(process));
  }

  template <int N>
  static void AddNormalMeasurement(
      Context& context,
      std::string name,
      const Identifier& variable,
      const beluga_estimation::Vector<DefaultScalar, N>& obs,
      const beluga_estimation::Matrix<DefaultScalar, N, N>& covariance) {
    auto measurement = std::make_shared<NormalLikelihoodConstraint_<DefaultScalar, N>>(
        Identifier{std::move(name), variable.timestamp}, variable,
        std::make_shared<NormalDistribution_<DefaultScalar, N>>(obs, covariance));
    EXPECT_TRUE(context.dbn.Add(measurement));
  }

  static std::shared_ptr<const OnManifoldEstimate> GetEstimate(Context& context) {
    auto result = context.estimates.template GetLatest<OnManifoldEstimate>("main");
    EXPECT_TRUE(result);
    return result ? Unwrap(result) : nullptr;
  }

  static std::shared_ptr<SE2Variable> AddPoseVariable(Context& context, const std::string& name, TimePoint time) {
    auto variable = std::make_shared<SE2Variable>(Identifier{name, time});
    EXPECT_TRUE(context.dbn.Add(variable));
    return variable;
  }

  static std::shared_ptr<EuclideanVariable>
  AddVelocityVariable(Context& context, const std::string& name, TimePoint time) {
    auto variable = std::make_shared<EuclideanVariable>(Identifier{name, time}, beluga_estimation::NDim<kVelTangent>{});
    EXPECT_TRUE(context.dbn.Add(variable));
    return variable;
  }

  static void AddPosePrior(
      Context& context,
      std::string name,
      const Identifier& variable,
      const beluga_estimation::SE2d& mean,
      const beluga_estimation::Matrix3d& covariance) {
    EXPECT_TRUE(context.dbn.Add(std::make_shared<SE2NormalPosePriorConstraint>(
        Identifier{std::move(name), variable.timestamp}, variable, SE2NormalDistribution(mean, covariance))));
  }

  static void AddVelocityPrior(
      Context& context,
      std::string name,
      const Identifier& variable,
      const beluga_estimation::Vector3d& mean,
      const beluga_estimation::Matrix3d& covariance) {
    AddNormalPrior<kVelTangent>(context, std::move(name), variable, mean, covariance);
  }

  static void AddKinematicProcess(
      Context& context,
      std::string name,
      const Identifier& pose_from,
      const Identifier& vel_from,
      const Identifier& pose_to,
      const Identifier& vel_to,
      const beluga_estimation::Matrix3d& pose_covariance,
      const beluga_estimation::Matrix3d& velocity_covariance) {
    EXPECT_TRUE(context.dbn.Add(std::make_shared<SE2ConstantVelocityKinematicConstraint>(
        Identifier{std::move(name), pose_to.timestamp}, pose_from, vel_from, pose_to, vel_to, pose_covariance,
        velocity_covariance)));
  }

  static void AddPoseMeasurement(
      Context& context,
      std::string name,
      const Identifier& variable,
      const beluga_estimation::SE2d& mean,
      const beluga_estimation::Matrix3d& covariance) {
    EXPECT_TRUE(context.dbn.Add(std::make_shared<SE2NormalPoseLikelihoodConstraint>(
        Identifier{std::move(name), variable.timestamp}, SE2NormalDistribution(mean, covariance), variable)));
  }

  static std::shared_ptr<const TangentDistribution> GetMarginalDistribution(
      const std::shared_ptr<const OnManifoldEstimate>& estimate,
      const Identifier& variable_id) {
    auto marginal_estimate = estimate->Marginalize(variable_id);
    EXPECT_NE(marginal_estimate, nullptr);
    if (!marginal_estimate) {
      return nullptr;
    }
    return marginal_estimate->GetDistribution();
  }

  Context context_;
};

TEST_F(ExtendedKalmanFilterTest, CanInstantiateConfigurationStruct) {
  ExtendedKalmanFilter::Configuration config;
  config.tracked_variable_names = {"pose", "velocity"};
  config.estimate_name = "ekf_estimate";

  EXPECT_EQ(config.tracked_variable_names.size(), 2u);
  EXPECT_EQ(config.tracked_variable_names[0], "pose");
  EXPECT_EQ(config.tracked_variable_names[1], "velocity");
  EXPECT_EQ(config.estimate_name, "ekf_estimate");
}

TEST_F(ExtendedKalmanFilterTest, DefaultEstimateNameIsMain) {
  ExtendedKalmanFilter::Configuration config;
  EXPECT_EQ(config.estimate_name, "main");
}

TEST_F(ExtendedKalmanFilterTest, DefaultTrackedVariableNamesIsEmpty) {
  ExtendedKalmanFilter::Configuration config;
  EXPECT_TRUE(config.tracked_variable_names.empty());
}

TEST_F(ExtendedKalmanFilterTest, ConstructionWithExplicitConfig) {
  EXPECT_NO_THROW({ MakeFilter({"pose"}); });
}

TEST_F(ExtendedKalmanFilterTest, ConstructionFromParams) {
  auto middleware = std::make_shared<MockMiddleware>();

  EXPECT_CALL(*middleware->parameters, DeclareParameter("test.tracked.variables", testing::An<const ParameterValue&>()))
      .WillOnce(Return(ParameterValue{std::vector<std::string>{"pose"}}));
  EXPECT_CALL(*middleware->parameters, DeclareParameter("test.estimate.name", testing::An<const ParameterValue&>()))
      .WillOnce(Return(ParameterValue{std::string{"main"}}));

  EXPECT_NO_THROW({ ExtendedKalmanFilter ekf(middleware, "test."); });
}

TEST_F(ExtendedKalmanFilterTest, BootstrapFromSinglePrior) {
  const Vector2d prior_mean{1.0, 2.0};
  const Matrix2d prior_covariance = Matrix2d::Identity();

  auto variable = AddEuclideanVariable<2>(context_, "x", TimePoint{});
  AddNormalPrior(context_, "prior", variable->Id(), prior_mean, prior_covariance);

  auto ekf = MakeFilter({"x"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 1u);
  EXPECT_EQ(vars[0]->Id(), variable->Id());

  auto distribution = estimate->GetDistribution();
  ASSERT_NE(distribution, nullptr);
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);
  EXPECT_EQ(embedded->DomainSize(), 2u);

  Vector2d estimate_mean;
  distribution->ComputeMean(to_view(estimate_mean));
  EXPECT_NEAR(estimate_mean.x(), prior_mean.x(), kTolerance);
  EXPECT_NEAR(estimate_mean.y(), prior_mean.y(), kTolerance);

  Matrix2d estimate_covariance;
  embedded->ComputeCovariance(to_view(estimate_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(estimate_covariance));
  EXPECT_NEAR(estimate_covariance(0, 0), prior_covariance(0, 0), kTolerance);
  EXPECT_NEAR(estimate_covariance(1, 1), prior_covariance(1, 1), kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, BootstrapFromMultiplePriors) {
  const Vector3d prior_x_mean{1.0, 2.0, 3.0};
  const Vector2d prior_y_mean{4.0, 5.0};
  const Matrix3d prior_x_covariance = Matrix3d::Identity();
  const Matrix2d prior_y_covariance = 2.0 * Matrix2d::Identity();

  auto var_x = AddEuclideanVariable<3>(context_, "x", TimePoint{});
  auto var_y = AddEuclideanVariable<2>(context_, "y", TimePoint{});

  AddNormalPrior(context_, "prior_x", var_x->Id(), prior_x_mean, prior_x_covariance);
  AddNormalPrior(context_, "prior_y", var_y->Id(), prior_y_mean, prior_y_covariance);

  auto ekf = MakeFilter({"x", "y"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);

  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 2u);
  EXPECT_EQ(vars[0]->Id(), var_x->Id());
  EXPECT_EQ(vars[1]->Id(), var_y->Id());

  auto distribution = estimate->GetDistribution();
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);
  EXPECT_EQ(embedded->DomainSize(), 5u);

  VectorXd joint_mean(5);
  distribution->ComputeMean(to_view(joint_mean));
  EXPECT_NEAR(joint_mean(0), prior_x_mean(0), kTolerance);
  EXPECT_NEAR(joint_mean(1), prior_x_mean(1), kTolerance);
  EXPECT_NEAR(joint_mean(2), prior_x_mean(2), kTolerance);
  EXPECT_NEAR(joint_mean(3), prior_y_mean(0), kTolerance);
  EXPECT_NEAR(joint_mean(4), prior_y_mean(1), kTolerance);

  MatrixXd joint_covariance(5, 5);
  embedded->ComputeCovariance(to_view(joint_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(joint_covariance));
  EXPECT_NEAR(joint_covariance(0, 0), prior_x_covariance(0, 0), kTolerance);
  EXPECT_NEAR(joint_covariance(1, 1), prior_x_covariance(1, 1), kTolerance);
  EXPECT_NEAR(joint_covariance(2, 2), prior_x_covariance(2, 2), kTolerance);
  EXPECT_NEAR(joint_covariance(3, 3), prior_y_covariance(0, 0), kTolerance);
  EXPECT_NEAR(joint_covariance(4, 4), prior_y_covariance(1, 1), kTolerance);
  EXPECT_NEAR(joint_covariance(3, 0), 0.0, kTolerance);
  EXPECT_NEAR(joint_covariance(0, 3), 0.0, kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, BootstrapFailsWhenVariableNotFound) {
  auto ekf = MakeFilter({"nonexistent"});
  EXPECT_EQ(ekf->Execute(context_, ComputeBudget::Unlimited()).error(), Error::kFailure);
}

TEST_F(ExtendedKalmanFilterTest, BootstrapFailsWhenPriorMissing) {
  auto manifold = std::make_shared<MockManifold>();
  auto variable = std::make_shared<MockVariable>(Identifier{"x", TimePoint{}});
  EXPECT_CALL(*manifold, TangentSize()).WillRepeatedly(Return(2));
  EXPECT_CALL(*manifold, AmbientSize()).WillRepeatedly(Return(2));
  EXPECT_CALL(*variable, Manifold()).WillRepeatedly(Return(manifold));
  ASSERT_TRUE(context_.dbn.Add(variable));

  auto ekf = MakeFilter({"x"});
  EXPECT_EQ(ekf->Execute(context_, ComputeBudget::Unlimited()).error(), Error::kFailure);
  EXPECT_EQ(GetEstimate(context_), nullptr);
}

TEST_F(ExtendedKalmanFilterTest, ExecuteReplacesEstimateInStore) {
  const auto t0 = TimePoint{};
  const auto x0 = AddEuclideanVariable<2>(context_, "x", t0);
  const Vector2d prior_mean{1.0, 2.0};
  const Matrix2d prior_covariance = Matrix2d::Identity();
  AddNormalPrior(context_, "prior", x0->Id(), prior_mean, prior_covariance);

  auto stale_mean = Vector2d{9.0, 9.0};
  auto stale_covariance = Matrix2d::Identity();
  auto stale_normal = std::make_shared<NormalDistribution>(stale_mean, stale_covariance);
  auto stale_manifold = std::make_shared<EuclideanManifold>(2);
  auto stale_tangent = std::make_shared<GenericTangentDistribution>(stale_manifold, stale_normal);
  auto stale_estimate =
      std::make_shared<OnManifoldEstimate>(std::vector<std::shared_ptr<const Variable>>{x0}, stale_tangent);
  context_.estimates.Add(Identifier{"main", t0}, std::move(stale_estimate));

  auto ekf = MakeFilter({"x"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto distribution = estimate->GetDistribution();
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);

  Vector2d estimate_mean;
  distribution->ComputeMean(to_view(estimate_mean));
  EXPECT_NEAR(estimate_mean.x(), prior_mean.x(), kTolerance);
  EXPECT_NEAR(estimate_mean.y(), prior_mean.y(), kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, ExecuteFailsWhenNoTrackedVariables) {
  auto ekf = MakeFilter({});
  EXPECT_EQ(ekf->Execute(context_, ComputeBudget::Unlimited()).error(), Error::kFailure);
}

TEST_F(ExtendedKalmanFilterTest, ExecuteWithEmptyDBN) {
  auto ekf = MakeFilter({"x"});
  EXPECT_EQ(ekf->Execute(context_, ComputeBudget::Unlimited()).error(), Error::kFailure);
}

TEST_F(ExtendedKalmanFilterTest, PredictThenUpdate) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};
  const auto x0 = AddEuclideanVariable<2>(context_, "x", t0);
  const auto x1 = AddEuclideanVariable<2>(context_, "x", t1);

  const Vector2d prior_mean{1.0, 2.0};
  const Matrix2d prior_covariance = Matrix2d::Identity();
  const Vector2d process_mean = Vector2d::Zero();
  const Matrix2d process_covariance = Matrix2d::Identity();
  const Vector2d measurement_mean{2.0, 4.0};
  const Matrix2d measurement_covariance = 0.01 * Matrix2d::Identity();

  AddNormalPrior(context_, "prior", x0->Id(), prior_mean, prior_covariance);
  AddNormalDelta(context_, "process", x0->Id(), x1->Id(), process_mean, process_covariance);
  AddNormalMeasurement(context_, "measurement", x1->Id(), measurement_mean, measurement_covariance);

  auto ekf = MakeFilter({"x"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);

  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 1u);
  EXPECT_EQ(vars[0]->Id(), x1->Id());

  const Matrix2d prediction_covariance = prior_covariance + process_covariance;
  const Matrix2d innovation_covariance = prediction_covariance + measurement_covariance;
  const Matrix2d kalman_gain = prediction_covariance * innovation_covariance.inverse();
  const Vector2d expected_mean = prior_mean + process_mean + kalman_gain * (measurement_mean - prior_mean);
  const Matrix2d expected_covariance = (Matrix2d::Identity() - kalman_gain) * prediction_covariance;

  auto distribution = estimate->GetDistribution();
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);

  Vector2d mean;
  distribution->ComputeMean(to_view(mean));
  EXPECT_NEAR(mean(0), expected_mean(0), kTolerance);
  EXPECT_NEAR(mean(1), expected_mean(1), kTolerance);

  Matrix2d covariance;
  embedded->ComputeCovariance(to_view(covariance));
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
  EXPECT_NEAR(covariance(0, 0), expected_covariance(0, 0), kTolerance);
  EXPECT_NEAR(covariance(1, 1), expected_covariance(1, 1), kTolerance);
  EXPECT_NEAR(covariance(0, 1), expected_covariance(0, 1), kTolerance);
  EXPECT_NEAR(covariance(1, 0), expected_covariance(1, 0), kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, PredictThenUpdateWithSlicedMeasurement) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};
  const auto x0 = AddEuclideanVariable<3>(context_, "x", t0);
  const auto x1 = AddEuclideanVariable<3>(context_, "x", t1);

  const Vector3d prior_mean{1.0, 2.0, 0.5};
  const Matrix3d prior_covariance = Matrix3d::Identity();
  const Matrix3d process_covariance = 0.5 * Matrix3d::Identity();
  const Vector3d process_mean = Vector3d::Zero();
  const double observed = 2.0;
  const double measurement_variance = 0.01;

  AddNormalPrior(context_, "prior", x0->Id(), prior_mean, prior_covariance);
  AddNormalDelta(context_, "process", x0->Id(), x1->Id(), process_mean, process_covariance);
  // Observes only the third coordinate of the 3D variable.
  auto measurement_distribution = std::make_shared<const NormalDistribution_<double>>(
      Vector1d{{observed}}, Matrix1d::Identity() * measurement_variance);
  auto measurement = std::make_shared<NormalLikelihoodConstraint_<double>>(
      Identifier{"measurement", t1}, x1->Id(), measurement_distribution,
      (beluga_estimation::Matrix<double, 1, 3>() << 0.0, 0.0, 1.0).finished());
  ASSERT_TRUE(context_.dbn.Add(measurement));

  auto ekf = MakeFilter({"x"});
  ASSERT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  const Vector3d predicted_mean = prior_mean;
  const Matrix3d predicted_covariance = prior_covariance + process_covariance;
  const Vector3d h{0.0, 0.0, 1.0};
  const double innovation = observed - h.dot(predicted_mean);
  const double innovation_variance = h.dot(predicted_covariance * h) + measurement_variance;
  const Vector3d gain = predicted_covariance * h / innovation_variance;
  const Vector3d expected_mean = predicted_mean + gain * innovation;
  const Matrix3d expected_covariance = (Matrix3d::Identity() - gain * h.transpose()) * predicted_covariance;

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);

  auto distribution = estimate->GetDistribution();
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);

  Vector3d mean;
  distribution->ComputeMean(to_view(mean));
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(mean(i), expected_mean(i), kTolerance);
  }

  Matrix3d covariance;
  embedded->ComputeCovariance(to_view(covariance));
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(covariance(i, j), expected_covariance(i, j), kTolerance);
    }
  }
}

TEST_F(ExtendedKalmanFilterTest, PredictThenPartialUpdate) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};

  const auto x0 = AddEuclideanVariable<2>(context_, "x", t0);
  const auto x1 = AddEuclideanVariable<2>(context_, "x", t1);
  const auto y0 = AddEuclideanVariable<2>(context_, "y", t0);
  const auto y1 = AddEuclideanVariable<2>(context_, "y", t1);

  const Vector2d prior_x_mean{1.0, 2.0};
  const Matrix2d prior_x_covariance = Matrix2d::Identity();
  const Vector2d prior_y_mean = Vector2d::Zero();
  const Matrix2d prior_y_covariance = Matrix2d::Identity();
  const Vector2d process_mean = Vector2d::Zero();
  const Matrix2d process_covariance = Matrix2d::Identity();
  const Vector2d measurement_y_mean{2.0, 4.0};
  const Matrix2d measurement_y_covariance = 0.01 * Matrix2d::Identity();

  AddNormalPrior(context_, "prior_x", x0->Id(), prior_x_mean, prior_x_covariance);
  AddNormalPrior(context_, "prior_y", y0->Id(), prior_y_mean, prior_y_covariance);
  AddNormalDelta(context_, "proc_x", x0->Id(), x1->Id(), process_mean, process_covariance);
  AddNormalDelta(context_, "proc_y", y0->Id(), y1->Id(), process_mean, process_covariance);
  AddNormalMeasurement(context_, "measurement_y", y1->Id(), measurement_y_mean, measurement_y_covariance);

  auto ekf = MakeFilter({"x", "y"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);

  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 2u);
  EXPECT_EQ(vars[0]->Id().timestamp, t1);
  EXPECT_EQ(vars[1]->Id().timestamp, t1);

  const Matrix2d x_prediction_covariance = prior_x_covariance + process_covariance;
  const Matrix2d y_prediction_covariance = prior_y_covariance + process_covariance;
  const Matrix2d y_innovation_covariance = y_prediction_covariance + measurement_y_covariance;
  const Matrix2d y_kalman_gain = y_prediction_covariance * y_innovation_covariance.inverse();

  const Vector2d expected_x_mean = prior_x_mean + process_mean;
  const Vector2d expected_y_mean = prior_y_mean + process_mean + y_kalman_gain * (measurement_y_mean - prior_y_mean);

  const Matrix2d expected_x_covariance = x_prediction_covariance;
  const Matrix2d expected_y_covariance = (Matrix2d::Identity() - y_kalman_gain) * y_prediction_covariance;

  auto distribution = estimate->GetDistribution();
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);

  VectorXd mean(4);
  distribution->ComputeMean(to_view(mean));
  EXPECT_NEAR(mean(0), expected_x_mean(0), kTolerance);
  EXPECT_NEAR(mean(1), expected_x_mean(1), kTolerance);
  EXPECT_NEAR(mean(2), expected_y_mean(0), kTolerance);
  EXPECT_NEAR(mean(3), expected_y_mean(1), kTolerance);

  MatrixXd covariance(4, 4);
  embedded->ComputeCovariance(to_view(covariance));
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
  EXPECT_NEAR(covariance(0, 0), expected_x_covariance(0, 0), kTolerance);
  EXPECT_NEAR(covariance(1, 1), expected_x_covariance(1, 1), kTolerance);
  EXPECT_NEAR(covariance(2, 2), expected_y_covariance(0, 0), kTolerance);
  EXPECT_NEAR(covariance(3, 3), expected_y_covariance(1, 1), kTolerance);
  EXPECT_NEAR(covariance(0, 2), 0.0, kTolerance);
  EXPECT_NEAR(covariance(2, 0), 0.0, kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, PredictThenUpdateRepeatedly) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};
  const auto t2 = TimePoint{std::chrono::seconds(2)};

  const auto x0 = AddEuclideanVariable<1>(context_, "x", t0);
  const auto x1 = AddEuclideanVariable<1>(context_, "x", t1);
  const auto x2 = AddEuclideanVariable<1>(context_, "x", t2);

  const Vector1d prior_mean{0.0};
  const Matrix1d prior_covariance = Matrix1d::Identity();
  const Vector1d process_mean{1.0};
  const Matrix1d process_covariance = 0.25 * Matrix1d::Identity();
  const Vector1d measurement_1_mean{1.5};
  const Vector1d measurement_2_mean{3.0};
  const Matrix1d measurement_covariance = 0.25 * Matrix1d::Identity();

  AddNormalPrior(context_, "prior", x0->Id(), prior_mean, prior_covariance);
  AddNormalDelta(context_, "process_1", x0->Id(), x1->Id(), process_mean, process_covariance);
  AddNormalMeasurement(context_, "meas_1", x1->Id(), measurement_1_mean, measurement_covariance);
  AddNormalDelta(context_, "process_2", x1->Id(), x2->Id(), process_mean, process_covariance);
  AddNormalMeasurement(context_, "meas_2", x2->Id(), measurement_2_mean, measurement_covariance);

  auto ekf = MakeFilter({"x"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);

  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 1u);
  EXPECT_EQ(vars[0]->Id(), x2->Id());

  const Matrix1d prediction_1_covariance = prior_covariance + process_covariance;
  const Matrix1d innovation_1_covariance = prediction_1_covariance + measurement_covariance;
  const Matrix1d kalman_1_gain = prediction_1_covariance * innovation_1_covariance.inverse();
  const Vector1d predicted_1_mean = prior_mean + process_mean;
  const Vector1d expected_1_mean = predicted_1_mean + kalman_1_gain * (measurement_1_mean - predicted_1_mean);
  const Matrix1d expected_1_covariance = (Matrix1d::Identity() - kalman_1_gain) * prediction_1_covariance;

  const Matrix1d prediction_2_covariance = expected_1_covariance + process_covariance;
  const Matrix1d innovation_2_covariance = prediction_2_covariance + measurement_covariance;
  const Matrix1d kalman_2_gain = prediction_2_covariance * innovation_2_covariance.inverse();
  const Vector1d predicted_2_mean = expected_1_mean + process_mean;
  const Vector1d expected_2_mean = predicted_2_mean + kalman_2_gain * (measurement_2_mean - predicted_2_mean);
  const Matrix1d expected_2_covariance = (Matrix1d::Identity() - kalman_2_gain) * prediction_2_covariance;

  auto distribution = estimate->GetDistribution();
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);

  Vector1d mean;
  distribution->ComputeMean(to_view(mean));
  EXPECT_NEAR(mean.value(), expected_2_mean.value(), kTolerance);

  Matrix1d covariance;
  embedded->ComputeCovariance(to_view(covariance));
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
  EXPECT_NEAR(covariance.value(), expected_2_covariance.value(), kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, PredictThenUpdateGreedily) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};
  const auto t2 = TimePoint{std::chrono::seconds(2)};

  const auto x0 = AddEuclideanVariable<1>(context_, "x", t0);
  const auto x1 = AddEuclideanVariable<1>(context_, "x", t1);
  const auto x2 = AddEuclideanVariable<1>(context_, "x", t2);
  const auto y0 = AddEuclideanVariable<1>(context_, "y", t0);
  const auto y1 = AddEuclideanVariable<1>(context_, "y", t1);

  const Vector1d prior_mean{0.0};
  const Matrix1d prior_covariance = Matrix1d::Identity();
  const Vector1d process_mean{1.0};
  const Matrix1d process_covariance = 0.25 * Matrix1d::Identity();
  const Vector1d measurement_mean{1.5};
  const Matrix1d measurement_covariance = 0.25 * Matrix1d::Identity();

  AddNormalPrior(context_, "prior_x", x0->Id(), prior_mean, prior_covariance);
  AddNormalPrior(context_, "prior_y", y0->Id(), prior_mean, prior_covariance);
  AddNormalDelta(context_, "proc_x_1", x0->Id(), x1->Id(), process_mean, process_covariance);
  AddNormalDelta(context_, "proc_y_1", y0->Id(), y1->Id(), process_mean, process_covariance);
  AddNormalDelta(context_, "proc_x_2", x1->Id(), x2->Id(), process_mean, process_covariance);
  AddNormalMeasurement(context_, "meas_x_1", x1->Id(), measurement_mean, measurement_covariance);
  AddNormalMeasurement(context_, "meas_y_1", y1->Id(), measurement_mean, measurement_covariance);

  auto ekf = MakeFilter({"x", "y"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);

  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 2u);
  EXPECT_EQ(vars[0]->Id(), x1->Id());
  EXPECT_EQ(vars[1]->Id(), y1->Id());

  const Matrix1d prediction_covariance = prior_covariance + process_covariance;
  const Matrix1d innovation_covariance = prediction_covariance + measurement_covariance;
  const Matrix1d kalman_gain = prediction_covariance * innovation_covariance.inverse();
  const Vector1d predicted_mean = prior_mean + process_mean;
  const Vector1d expected_mean = predicted_mean + kalman_gain * (measurement_mean - predicted_mean);
  const Matrix1d expected_covariance = (Matrix1d::Identity() - kalman_gain) * prediction_covariance;

  auto distribution = estimate->GetDistribution();
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);

  VectorXd mean(2);
  distribution->ComputeMean(to_view(mean));
  EXPECT_NEAR(mean(0), expected_mean.value(), kTolerance);
  EXPECT_NEAR(mean(1), expected_mean.value(), kTolerance);

  MatrixXd covariance(2, 2);
  embedded->ComputeCovariance(to_view(covariance));
  EXPECT_TRUE(IsCovarianceMatrix(covariance));
  EXPECT_NEAR(covariance(0, 0), expected_covariance.value(), kTolerance);
  EXPECT_NEAR(covariance(1, 1), expected_covariance.value(), kTolerance);
  EXPECT_NEAR(covariance(0, 1), 0.0, kTolerance);
  EXPECT_NEAR(covariance(1, 0), 0.0, kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, NoOpWhenEstimateAtLatestTimestep) {
  const auto t0 = TimePoint{};
  const auto x0 = AddEuclideanVariable<2>(context_, "x", t0);
  const Vector2d prior_mean{1.0, 1.0};
  const Matrix2d prior_covariance = Matrix2d::Identity();
  AddNormalPrior(context_, "prior", x0->Id(), prior_mean, prior_covariance);

  const Vector2d process_mean{1.0, 1.0};
  const Matrix2d process_covariance = Matrix2d::Identity();
  const auto t1 = TimePoint{std::chrono::seconds(1)};
  const auto x1 = AddEuclideanVariable<2>(context_, "x", t1);
  AddNormalDelta(context_, "process", x0->Id(), x1->Id(), process_mean, process_covariance);

  const Vector2d measurement_mean{1.0, 1.0};
  const Matrix2d measurement_covariance = Matrix2d::Identity();
  AddNormalMeasurement(context_, "measurement", x1->Id(), measurement_mean, measurement_covariance);

  auto ekf = MakeFilter({"x"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));
  auto first_estimate = GetEstimate(context_);

  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));
  auto second_estimate = GetEstimate(context_);

  EXPECT_EQ(first_estimate, second_estimate);
}

TEST_F(ExtendedKalmanFilterTest, TruncatesToShortestSequence) {
  const auto t0 = TimePoint{};
  const auto x0 = AddEuclideanVariable<1>(context_, "x", t0);
  const auto y0 = AddEuclideanVariable<1>(context_, "y", t0);

  const Vector1d zero_mean{0.0};
  const Matrix1d standard_covariance = Matrix1d::Identity();
  AddNormalPrior(context_, "prior_x", x0->Id(), zero_mean, standard_covariance);
  AddNormalPrior(context_, "prior_y", y0->Id(), zero_mean, standard_covariance);

  const auto t1 = TimePoint{std::chrono::seconds(1)};
  AddEuclideanVariable<1>(context_, "x", t1);

  auto ekf = MakeFilter({"x", "y"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 2u);
  EXPECT_EQ(vars[0]->Id().timestamp, t0);
  EXPECT_EQ(vars[1]->Id().timestamp, t0);
}

TEST_F(ExtendedKalmanFilterTest, ProcessesNewChunksAddedAfterFirstCall) {
  const auto t0 = TimePoint{};
  const auto x0 = AddEuclideanVariable<1>(context_, "x", t0);
  const Vector1d prior_mean{5.0};
  const Matrix1d prior_covariance = Matrix1d::Identity();
  AddNormalPrior(context_, "prior", x0->Id(), prior_mean, prior_covariance);

  auto ekf = MakeFilter({"x"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  const auto t1 = TimePoint{std::chrono::seconds(1)};
  const auto x1 = AddEuclideanVariable<1>(context_, "x", t1);
  const Vector1d process_mean{2.0};
  const Matrix1d process_covariance = Matrix1d::Identity();
  AddNormalDelta(context_, "process", x0->Id(), x1->Id(), process_mean, process_covariance);

  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 1u);
  EXPECT_EQ(vars[0]->Id().timestamp, t1);

  auto distribution = estimate->GetDistribution();
  auto embedded = distribution->GetEmbeddedDistribution();
  ASSERT_NE(embedded, nullptr);

  Vector1d mean;
  distribution->ComputeMean(to_view(mean));
  EXPECT_NEAR(mean(0), process_mean(0) + prior_mean(0), kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, FailsWhenConstraintReferencesUntrackedVariable) {
  TimePoint t0{};
  TimePoint t1{std::chrono::seconds(1)};

  auto x0 = AddEuclideanVariable<1>(context_, "x", t0);
  auto x1 = AddEuclideanVariable<1>(context_, "x", t1);
  auto z0 = AddEuclideanVariable<1>(context_, "z", t0);
  const auto id_x0 = x0->Id();
  const auto id_z0 = z0->Id();
  const Vector1d zero_mean{0.0};
  const Matrix1d standard_covariance = Matrix1d::Identity();

  AddNormalPrior(context_, "prior", x0->Id(), zero_mean, standard_covariance);
  AddNormalDelta(context_, "process", z0->Id(), x1->Id(), zero_mean, standard_covariance);

  auto ekf = MakeFilter({"x"});
  EXPECT_EQ(ekf->Execute(context_, ComputeBudget::Unlimited()).error(), Error::kFailure);
}

TEST_F(ExtendedKalmanFilterTest, MixedManifoldBootstrapFromSinglePriors) {
  const auto t0 = TimePoint{};
  const auto pose0 = AddPoseVariable(context_, "pose", t0);
  const auto vel0 = AddVelocityVariable(context_, "vel", t0);

  const SE2d prior_pose{SO2d{0.2}, Vector2d{1.0, 2.0}};
  const Vector3d prior_vel{0.1, -0.2, 0.05};
  const Matrix3d prior_pose_covariance = 0.1 * Matrix3d::Identity();
  const Matrix3d prior_vel_covariance = 0.2 * Matrix3d::Identity();

  AddPosePrior(context_, "prior_pose", pose0->Id(), prior_pose, prior_pose_covariance);
  AddVelocityPrior(context_, "prior_vel", vel0->Id(), prior_vel, prior_vel_covariance);

  auto ekf = MakeFilter({"pose", "vel"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 2u);
  EXPECT_EQ(vars[0]->Id(), pose0->Id());
  EXPECT_EQ(vars[1]->Id(), vel0->Id());

  auto pose_distribution = GetMarginalDistribution(estimate, pose0->Id());
  ASSERT_NE(pose_distribution, nullptr);
  SE2d pose_mean;
  pose_distribution->ComputeMean(to_view(pose_mean));
  EXPECT_NEAR((pose_mean.inverse() * prior_pose).log().norm(), 0.0, kTolerance);
  Matrix3d pose_covariance;
  pose_distribution->GetEmbeddedDistribution()->ComputeCovariance(to_view(pose_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(pose_covariance));
  EXPECT_NEAR((pose_covariance - prior_pose_covariance).norm(), 0.0, kTolerance);

  auto vel_distribution = GetMarginalDistribution(estimate, vel0->Id());
  ASSERT_NE(vel_distribution, nullptr);
  Vector3d vel_mean;
  vel_distribution->ComputeMean(to_view(vel_mean));
  EXPECT_NEAR((vel_mean - prior_vel).norm(), 0.0, kTolerance);
  Matrix3d vel_covariance;
  vel_distribution->ComputeCovariance(to_view(vel_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(vel_covariance));
  EXPECT_NEAR((vel_covariance - prior_vel_covariance).norm(), 0.0, kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, MixedManifoldBootstrapFromMultiplePriors) {
  constexpr std::size_t kBiasAmbient = 2;
  constexpr std::size_t kBiasTangent = 2;

  const auto t0 = TimePoint{};
  const auto pose0 = AddPoseVariable(context_, "pose", t0);
  const auto vel0 = AddVelocityVariable(context_, "vel", t0);
  const auto bias0 = AddEuclideanVariable<2>(context_, "bias", t0);

  const SE2d prior_pose{SO2d{0.1}, Vector2d{0.5, -1.0}};
  const Vector3d prior_vel{0.2, 0.1, -0.05};
  const Vector2d prior_bias{1.0, -2.0};
  const Matrix3d prior_pose_covariance = 0.1 * Matrix3d::Identity();
  const Matrix3d prior_vel_covariance = 0.2 * Matrix3d::Identity();
  const Matrix2d prior_bias_covariance = 0.3 * Matrix2d::Identity();

  AddPosePrior(context_, "prior_pose", pose0->Id(), prior_pose, prior_pose_covariance);
  AddVelocityPrior(context_, "prior_vel", vel0->Id(), prior_vel, prior_vel_covariance);
  AddNormalPrior<2>(context_, "prior_bias", bias0->Id(), prior_bias, prior_bias_covariance);

  auto ekf = MakeFilter({"pose", "vel", "bias"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 3u);
  EXPECT_EQ(vars[0]->Id(), pose0->Id());
  EXPECT_EQ(vars[1]->Id(), vel0->Id());
  EXPECT_EQ(vars[2]->Id(), bias0->Id());

  auto pose_distribution = GetMarginalDistribution(estimate, pose0->Id());
  ASSERT_NE(pose_distribution, nullptr);
  SE2d pose_mean;
  pose_distribution->ComputeMean(to_view(pose_mean));
  EXPECT_NEAR((pose_mean.inverse() * prior_pose).log().norm(), 0.0, kTolerance);
  Matrix3d pose_covariance;
  pose_distribution->GetEmbeddedDistribution()->ComputeCovariance(to_view(pose_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(pose_covariance));
  EXPECT_NEAR((pose_covariance - prior_pose_covariance).norm(), 0.0, kTolerance);

  auto vel_distribution = GetMarginalDistribution(estimate, vel0->Id());
  ASSERT_NE(vel_distribution, nullptr);
  Vector3d vel_mean;
  vel_distribution->ComputeMean(to_view(vel_mean));
  EXPECT_NEAR((vel_mean - prior_vel).norm(), 0.0, kTolerance);
  Matrix3d vel_covariance;
  vel_distribution->ComputeCovariance(to_view(vel_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(vel_covariance));
  EXPECT_NEAR((vel_covariance - prior_vel_covariance).norm(), 0.0, kTolerance);

  auto bias_distribution = GetMarginalDistribution(estimate, bias0->Id());
  ASSERT_NE(bias_distribution, nullptr);
  Vector2d bias_mean;
  bias_distribution->ComputeMean(to_view(bias_mean));
  EXPECT_NEAR((bias_mean - prior_bias).norm(), 0.0, kTolerance);
  Matrix2d bias_covariance;
  bias_distribution->ComputeCovariance(to_view(bias_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(bias_covariance));
  EXPECT_NEAR((bias_covariance - prior_bias_covariance).norm(), 0.0, kTolerance);
}

TEST_F(ExtendedKalmanFilterTest, MixedManifoldPredictThenUpdate) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};

  const auto pose0 = AddPoseVariable(context_, "pose", t0);
  const auto vel0 = AddVelocityVariable(context_, "vel", t0);
  const auto pose1 = AddPoseVariable(context_, "pose", t1);
  const auto vel1 = AddVelocityVariable(context_, "vel", t1);

  const SE2d prior_pose{SO2d{0.0}, Vector2d{1.0, 2.0}};
  const Vector3d prior_vel{0.2, 0.0, 0.1};
  const Matrix3d prior_pose_covariance = 0.1 * Matrix3d::Identity();
  const Matrix3d prior_vel_covariance = 0.1 * Matrix3d::Identity();
  const Matrix3d process_covariance = 0.05 * Matrix3d::Identity();
  const SE2d measurement_pose{SO2d{0.35}, Vector2d{1.6, 2.3}};
  const Matrix3d measurement_pose_covariance = 0.01 * Matrix3d::Identity();

  AddPosePrior(context_, "prior_pose", pose0->Id(), prior_pose, prior_pose_covariance);
  AddVelocityPrior(context_, "prior_vel", vel0->Id(), prior_vel, prior_vel_covariance);
  AddKinematicProcess(
      context_, "process", pose0->Id(), vel0->Id(), pose1->Id(), vel1->Id(), process_covariance, process_covariance);
  AddPoseMeasurement(context_, "meas_pose", pose1->Id(), measurement_pose, measurement_pose_covariance);

  auto ekf = MakeFilter({"pose", "vel"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 2u);
  EXPECT_EQ(vars[0]->Id(), pose1->Id());
  EXPECT_EQ(vars[1]->Id(), vel1->Id());

  // Predict step expectations.
  const SE2d delta = SE2d::exp(prior_vel);
  const SE2d predicted_pose = prior_pose * delta;
  const Vector3d predicted_vel = prior_vel;

  Matrix<DefaultScalar, 6, 6> P_prior = Matrix<DefaultScalar, 6, 6>::Zero();
  P_prior.template block<3, 3>(0, 0) = prior_pose_covariance;
  P_prior.template block<3, 3>(3, 3) = prior_vel_covariance;

  Matrix3d Jr = Matrix3d::Identity();
  const Vector3d w = prior_vel;
  Jr(0, 1) = w(2) / 2.0;
  Jr(1, 0) = -w(2) / 2.0;
  Jr(0, 2) = -w(1) / 2.0;
  Jr(1, 2) = w(0) / 2.0;

  Matrix<DefaultScalar, 6, 6> F = Matrix<DefaultScalar, 6, 6>::Zero();
  F.template block<3, 3>(0, 0) = delta.inverse().Adj();
  F.template block<3, 3>(0, 3) = Jr;
  F.template block<3, 3>(3, 3) = Matrix3d::Identity();

  Matrix<DefaultScalar, 6, 6> Q = Matrix<DefaultScalar, 6, 6>::Zero();
  Q.template block<3, 3>(0, 0) = process_covariance;
  Q.template block<3, 3>(3, 3) = process_covariance;

  const Matrix<DefaultScalar, 6, 6> P_pred = F * P_prior * F.transpose() + Q;

  // Update step expectations.
  const Vector3d residual = (predicted_pose.inverse() * measurement_pose).log();
  Matrix3d H_pose = Matrix3d::Identity();
  const double half_theta = residual(2) / 2.0;
  H_pose(0, 1) = half_theta;
  H_pose(1, 0) = -half_theta;
  H_pose(0, 2) = -residual(1) / 2.0;
  H_pose(1, 2) = residual(0) / 2.0;

  Matrix<DefaultScalar, 3, 6> H = Matrix<DefaultScalar, 3, 6>::Zero();
  H.template block<3, 3>(0, 0) = H_pose;

  const Matrix3d S = H * P_pred * H.transpose() + measurement_pose_covariance;
  const Matrix<DefaultScalar, 6, 3> K = P_pred * H.transpose() * S.inverse();
  const Vector6d correction = K * residual;

  const SE2d expected_pose = predicted_pose * SE2d::exp(correction.template head<3>());
  const Vector3d expected_vel = predicted_vel + correction.template tail<3>();

  const Matrix<DefaultScalar, 6, 6> I = Matrix<DefaultScalar, 6, 6>::Identity();
  const Matrix<DefaultScalar, 6, 6> expected_covariance =
      (I - K * H) * P_pred * (I - K * H).transpose() + K * measurement_pose_covariance * K.transpose();

  auto pose_distribution = GetMarginalDistribution(estimate, pose1->Id());
  ASSERT_NE(pose_distribution, nullptr);
  SE2d pose_mean;
  pose_distribution->ComputeMean(to_view(pose_mean));
  EXPECT_NEAR((pose_mean.inverse() * expected_pose).log().norm(), 0.0, kTolerance);

  Matrix3d pose_covariance;
  pose_distribution->GetEmbeddedDistribution()->ComputeCovariance(to_view(pose_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(pose_covariance));
  for (Eigen::Index i = 0; i < 3; ++i) {
    for (Eigen::Index j = 0; j < 3; ++j) {
      EXPECT_NEAR(pose_covariance(i, j), expected_covariance(i, j), kTolerance);
    }
  }

  auto vel_distribution = GetMarginalDistribution(estimate, vel1->Id());
  ASSERT_NE(vel_distribution, nullptr);
  Vector3d vel_mean;
  vel_distribution->ComputeMean(to_view(vel_mean));
  EXPECT_NEAR((vel_mean - expected_vel).norm(), 0.0, kTolerance);

  Matrix3d vel_covariance;
  vel_distribution->ComputeCovariance(to_view(vel_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(vel_covariance));
  for (Eigen::Index i = 0; i < 3; ++i) {
    for (Eigen::Index j = 0; j < 3; ++j) {
      EXPECT_NEAR(vel_covariance(i, j), expected_covariance(i + 3, j + 3), kTolerance);
    }
  }
}

TEST_F(ExtendedKalmanFilterTest, MixedManifoldPredictThenPartialUpdate) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};

  const auto pose0 = AddPoseVariable(context_, "pose", t0);
  const auto vel0 = AddVelocityVariable(context_, "vel", t0);
  const auto pose1 = AddPoseVariable(context_, "pose", t1);
  const auto vel1 = AddVelocityVariable(context_, "vel", t1);

  const SE2d prior_pose{SO2d{0.0}, Vector2d{1.0, 2.0}};
  const Vector3d prior_vel{0.15, 0.0, 0.05};
  const Matrix3d prior_pose_covariance = 0.1 * Matrix3d::Identity();
  const Matrix3d prior_vel_covariance = 0.1 * Matrix3d::Identity();
  const Matrix3d process_covariance = 0.05 * Matrix3d::Identity();
  const SE2d measurement_pose{SO2d{0.2}, Vector2d{1.4, 2.2}};
  const Matrix3d measurement_pose_covariance = 0.01 * Matrix3d::Identity();

  AddPosePrior(context_, "prior_pose", pose0->Id(), prior_pose, prior_pose_covariance);
  AddVelocityPrior(context_, "prior_vel", vel0->Id(), prior_vel, prior_vel_covariance);
  AddKinematicProcess(
      context_, "process", pose0->Id(), vel0->Id(), pose1->Id(), vel1->Id(), process_covariance, process_covariance);
  AddPoseMeasurement(context_, "meas_pose", pose1->Id(), measurement_pose, measurement_pose_covariance);

  auto ekf = MakeFilter({"pose", "vel"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 2u);
  EXPECT_EQ(vars[0]->Id(), pose1->Id());
  EXPECT_EQ(vars[1]->Id(), vel1->Id());

  // Predict step expectations.
  const SE2d delta = SE2d::exp(prior_vel);
  const SE2d predicted_pose = prior_pose * delta;
  const Vector3d predicted_vel = prior_vel;

  Matrix<DefaultScalar, 6, 6> P_prior = Matrix<DefaultScalar, 6, 6>::Zero();
  P_prior.template block<3, 3>(0, 0) = prior_pose_covariance;
  P_prior.template block<3, 3>(3, 3) = prior_vel_covariance;

  Matrix3d Jr = Matrix3d::Identity();
  const Vector3d w = prior_vel;
  Jr(0, 1) = w(2) / 2.0;
  Jr(1, 0) = -w(2) / 2.0;
  Jr(0, 2) = -w(1) / 2.0;
  Jr(1, 2) = w(0) / 2.0;

  Matrix<DefaultScalar, 6, 6> F = Matrix<DefaultScalar, 6, 6>::Zero();
  F.template block<3, 3>(0, 0) = delta.inverse().Adj();
  F.template block<3, 3>(0, 3) = Jr;
  F.template block<3, 3>(3, 3) = Matrix3d::Identity();

  Matrix<DefaultScalar, 6, 6> Q = Matrix<DefaultScalar, 6, 6>::Zero();
  Q.template block<3, 3>(0, 0) = process_covariance;
  Q.template block<3, 3>(3, 3) = process_covariance;
  const Matrix<DefaultScalar, 6, 6> P_pred = F * P_prior * F.transpose() + Q;

  // Partial update with pose measurement only.
  const Vector3d residual = (predicted_pose.inverse() * measurement_pose).log();
  Matrix3d H_pose = Matrix3d::Identity();
  const double half_theta = residual(2) / 2.0;
  H_pose(0, 1) = half_theta;
  H_pose(1, 0) = -half_theta;
  H_pose(0, 2) = -residual(1) / 2.0;
  H_pose(1, 2) = residual(0) / 2.0;

  Matrix<DefaultScalar, 3, 6> H = Matrix<DefaultScalar, 3, 6>::Zero();
  H.template block<3, 3>(0, 0) = H_pose;
  const Matrix3d S = H * P_pred * H.transpose() + measurement_pose_covariance;
  const Matrix<DefaultScalar, 6, 3> K = P_pred * H.transpose() * S.inverse();
  const Vector6d correction = K * residual;
  const SE2d expected_pose = predicted_pose * SE2d::exp(correction.template head<3>());
  const Vector3d expected_vel = predicted_vel + correction.template tail<3>();
  const Matrix<DefaultScalar, 6, 6> I = Matrix<DefaultScalar, 6, 6>::Identity();
  const Matrix<DefaultScalar, 6, 6> expected_covariance =
      (I - K * H) * P_pred * (I - K * H).transpose() + K * measurement_pose_covariance * K.transpose();

  auto pose_distribution = GetMarginalDistribution(estimate, pose1->Id());
  ASSERT_NE(pose_distribution, nullptr);
  SE2d pose_mean;
  pose_distribution->ComputeMean(to_view(pose_mean));
  EXPECT_NEAR((pose_mean.inverse() * expected_pose).log().norm(), 0.0, kTolerance);
  Matrix3d pose_covariance;
  pose_distribution->GetEmbeddedDistribution()->ComputeCovariance(to_view(pose_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(pose_covariance));
  for (Eigen::Index i = 0; i < 3; ++i) {
    for (Eigen::Index j = 0; j < 3; ++j) {
      EXPECT_NEAR(pose_covariance(i, j), expected_covariance(i, j), kTolerance);
    }
  }

  auto vel_distribution = GetMarginalDistribution(estimate, vel1->Id());
  ASSERT_NE(vel_distribution, nullptr);
  Vector3d vel_mean;
  vel_distribution->ComputeMean(to_view(vel_mean));
  EXPECT_NEAR((vel_mean - expected_vel).norm(), 0.0, kTolerance);
  Matrix3d vel_covariance;
  vel_distribution->ComputeCovariance(to_view(vel_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(vel_covariance));
  for (Eigen::Index i = 0; i < 3; ++i) {
    for (Eigen::Index j = 0; j < 3; ++j) {
      EXPECT_NEAR(vel_covariance(i, j), expected_covariance(i + 3, j + 3), kTolerance);
    }
  }
}

TEST_F(ExtendedKalmanFilterTest, MixedManifoldPredictThenUpdateRepeatedly) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};
  const auto t2 = TimePoint{std::chrono::seconds(2)};

  const auto pose0 = AddPoseVariable(context_, "pose", t0);
  const auto vel0 = AddVelocityVariable(context_, "vel", t0);
  const auto pose1 = AddPoseVariable(context_, "pose", t1);
  const auto vel1 = AddVelocityVariable(context_, "vel", t1);
  const auto pose2 = AddPoseVariable(context_, "pose", t2);
  const auto vel2 = AddVelocityVariable(context_, "vel", t2);

  const SE2d prior_pose{SO2d{0.0}, Vector2d{0.0, 0.0}};
  const Vector3d prior_vel{0.2, 0.0, 0.1};
  const Matrix3d prior_pose_covariance = 0.1 * Matrix3d::Identity();
  const Matrix3d prior_vel_covariance = 0.1 * Matrix3d::Identity();
  const Matrix3d process_covariance = 0.05 * Matrix3d::Identity();
  const SE2d measurement_pose_1{SO2d{0.15}, Vector2d{0.25, 0.05}};
  const SE2d measurement_pose_2{SO2d{0.35}, Vector2d{0.55, 0.08}};
  const Matrix3d measurement_covariance = 0.01 * Matrix3d::Identity();

  AddPosePrior(context_, "prior_pose", pose0->Id(), prior_pose, prior_pose_covariance);
  AddVelocityPrior(context_, "prior_vel", vel0->Id(), prior_vel, prior_vel_covariance);

  AddKinematicProcess(
      context_, "process_1", pose0->Id(), vel0->Id(), pose1->Id(), vel1->Id(), process_covariance, process_covariance);
  AddPoseMeasurement(context_, "meas_pose_1", pose1->Id(), measurement_pose_1, measurement_covariance);

  AddKinematicProcess(
      context_, "process_2", pose1->Id(), vel1->Id(), pose2->Id(), vel2->Id(), process_covariance, process_covariance);
  AddPoseMeasurement(context_, "meas_pose_2", pose2->Id(), measurement_pose_2, measurement_covariance);

  auto ekf = MakeFilter({"pose", "vel"});
  EXPECT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  auto estimate = GetEstimate(context_);
  ASSERT_NE(estimate, nullptr);
  auto vars = estimate->GetVariables();
  ASSERT_EQ(ranges::size(vars), 2u);
  EXPECT_EQ(vars[0]->Id(), pose2->Id());
  EXPECT_EQ(vars[1]->Id(), vel2->Id());

  auto process_jacobian = [](const Vector3d& velocity) {
    const SE2d delta = SE2d::exp(velocity);
    Matrix3d Jr = Matrix3d::Identity();
    const Vector3d w = velocity;
    Jr(0, 1) = w(2) / 2.0;
    Jr(1, 0) = -w(2) / 2.0;
    Jr(0, 2) = -w(1) / 2.0;
    Jr(1, 2) = w(0) / 2.0;

    Matrix<DefaultScalar, 6, 6> F = Matrix<DefaultScalar, 6, 6>::Zero();
    F.template block<3, 3>(0, 0) = delta.inverse().Adj();
    F.template block<3, 3>(0, 3) = Jr;
    F.template block<3, 3>(3, 3) = Matrix3d::Identity();
    return F;
  };

  auto update_jacobian = [](const Vector3d& residual) {
    Matrix3d H_pose = Matrix3d::Identity();
    const double half_theta = residual(2) / 2.0;
    H_pose(0, 1) = half_theta;
    H_pose(1, 0) = -half_theta;
    H_pose(0, 2) = -residual(1) / 2.0;
    H_pose(1, 2) = residual(0) / 2.0;

    Matrix<DefaultScalar, 3, 6> H = Matrix<DefaultScalar, 3, 6>::Zero();
    H.template block<3, 3>(0, 0) = H_pose;
    return H;
  };

  Matrix<DefaultScalar, 6, 6> Q = Matrix<DefaultScalar, 6, 6>::Zero();
  Q.template block<3, 3>(0, 0) = process_covariance;
  Q.template block<3, 3>(3, 3) = process_covariance;

  // Step 1
  Matrix<DefaultScalar, 6, 6> P0 = Matrix<DefaultScalar, 6, 6>::Zero();
  P0.template block<3, 3>(0, 0) = prior_pose_covariance;
  P0.template block<3, 3>(3, 3) = prior_vel_covariance;

  const SE2d delta1 = SE2d::exp(prior_vel);
  const SE2d predicted_pose_1 = prior_pose * delta1;
  const Vector3d predicted_vel_1 = prior_vel;
  const Matrix<DefaultScalar, 6, 6> F1 = process_jacobian(prior_vel);
  const Matrix<DefaultScalar, 6, 6> P_pred_1 = F1 * P0 * F1.transpose() + Q;

  const Vector3d residual_1 = (predicted_pose_1.inverse() * measurement_pose_1).log();
  const Matrix<DefaultScalar, 3, 6> H1 = update_jacobian(residual_1);
  const Matrix3d S1 = H1 * P_pred_1 * H1.transpose() + measurement_covariance;
  const Matrix<DefaultScalar, 6, 3> K1 = P_pred_1 * H1.transpose() * S1.inverse();
  const Vector6d correction_1 = K1 * residual_1;

  const SE2d expected_pose_1 = predicted_pose_1 * SE2d::exp(correction_1.template head<3>());
  const Vector3d expected_vel_1 = predicted_vel_1 + correction_1.template tail<3>();

  const Matrix<DefaultScalar, 6, 6> I = Matrix<DefaultScalar, 6, 6>::Identity();
  const Matrix<DefaultScalar, 6, 6> P1 =
      (I - K1 * H1) * P_pred_1 * (I - K1 * H1).transpose() + K1 * measurement_covariance * K1.transpose();

  // Step 2
  const SE2d delta2 = SE2d::exp(expected_vel_1);
  const SE2d predicted_pose_2 = expected_pose_1 * delta2;
  const Vector3d predicted_vel_2 = expected_vel_1;
  const Matrix<DefaultScalar, 6, 6> F2 = process_jacobian(expected_vel_1);
  const Matrix<DefaultScalar, 6, 6> P_pred_2 = F2 * P1 * F2.transpose() + Q;

  const Vector3d residual_2 = (predicted_pose_2.inverse() * measurement_pose_2).log();
  const Matrix<DefaultScalar, 3, 6> H2 = update_jacobian(residual_2);
  const Matrix3d S2 = H2 * P_pred_2 * H2.transpose() + measurement_covariance;
  const Matrix<DefaultScalar, 6, 3> K2 = P_pred_2 * H2.transpose() * S2.inverse();
  const Vector6d correction_2 = K2 * residual_2;

  const SE2d expected_pose_2 = predicted_pose_2 * SE2d::exp(correction_2.template head<3>());
  const Vector3d expected_vel_2 = predicted_vel_2 + correction_2.template tail<3>();
  const Matrix<DefaultScalar, 6, 6> expected_covariance =
      (I - K2 * H2) * P_pred_2 * (I - K2 * H2).transpose() + K2 * measurement_covariance * K2.transpose();

  auto pose_distribution = GetMarginalDistribution(estimate, pose2->Id());
  ASSERT_NE(pose_distribution, nullptr);
  SE2d pose_mean;
  pose_distribution->ComputeMean(to_view(pose_mean));
  EXPECT_NEAR((pose_mean.inverse() * expected_pose_2).log().norm(), 0.0, kTolerance);

  Matrix3d pose_covariance;
  pose_distribution->GetEmbeddedDistribution()->ComputeCovariance(to_view(pose_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(pose_covariance));
  for (Eigen::Index i = 0; i < 3; ++i) {
    for (Eigen::Index j = 0; j < 3; ++j) {
      EXPECT_NEAR(pose_covariance(i, j), expected_covariance(i, j), kTolerance);
    }
  }

  auto vel_distribution = GetMarginalDistribution(estimate, vel2->Id());
  ASSERT_NE(vel_distribution, nullptr);
  Vector3d vel_mean;
  vel_distribution->ComputeMean(to_view(vel_mean));
  EXPECT_NEAR((vel_mean - expected_vel_2).norm(), 0.0, kTolerance);

  Matrix3d vel_covariance;
  vel_distribution->ComputeCovariance(to_view(vel_covariance));
  EXPECT_TRUE(IsCovarianceMatrix(vel_covariance));
  for (Eigen::Index i = 0; i < 3; ++i) {
    for (Eigen::Index j = 0; j < 3; ++j) {
      EXPECT_NEAR(vel_covariance(i, j), expected_covariance(i + 3, j + 3), kTolerance);
    }
  }
}

TEST_F(ExtendedKalmanFilterTest, EmitsDebugDiagnostics) {
  const Vector2d prior_mean{1.0, 2.0};
  const Matrix2d prior_covariance = Matrix2d::Identity();

  auto variable = AddEuclideanVariable<2>(context_, "x", TimePoint{});
  AddNormalPrior(context_, "prior", variable->Id(), prior_mean, prior_covariance);

  auto middleware = std::make_shared<MockMiddleware>();
  std::vector<std::string> debug_messages;
  ON_CALL(*middleware->logger, Log(LogLevel::kDebug, testing::_))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });
  EXPECT_CALL(*middleware->logger, Log(LogLevel::kDebug, testing::_)).Times(testing::AnyNumber());

  ExtendedKalmanFilter::Configuration config;
  config.tracked_variable_names = {"x"};
  auto ekf = std::make_shared<ExtendedKalmanFilter>(middleware, config);

  ASSERT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  EXPECT_THAT(debug_messages, testing::Contains("ExtendedKalmanFilter: bootstrapped from 1 tracked variable(s)"));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("ExtendedKalmanFilter: published estimate")));
  EXPECT_THAT(
      debug_messages,
      testing::Contains(testing::HasSubstr("joint state has ambient size 2 and tangent size 2 with seed variables")));
  EXPECT_THAT(
      debug_messages, testing::Contains(testing::HasSubstr("bootstrap estimate at timestamp 0 has mean [1, 2]")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("no new aligned state chunk after timestamp 0")));
  EXPECT_THAT(
      debug_messages, testing::Contains(testing::HasSubstr("published estimate at timestamp 0 has mean [1, 2]")));
}

TEST_F(ExtendedKalmanFilterTest, EmitsDebugDiagnosticsForPredictionAndUpdate) {
  const auto t0 = TimePoint{};
  const auto t1 = TimePoint{std::chrono::seconds(1)};
  const auto x0 = AddEuclideanVariable<2>(context_, "x", t0);
  const auto x1 = AddEuclideanVariable<2>(context_, "x", t1);

  const Vector2d prior_mean{1.0, 2.0};
  const Matrix2d prior_covariance = Matrix2d::Identity();
  const Vector2d process_mean = Vector2d::Zero();
  const Matrix2d process_covariance = 0.5 * Matrix2d::Identity();
  const Vector2d measurement_mean{1.5, 2.5};
  const Matrix2d measurement_covariance = 0.1 * Matrix2d::Identity();

  AddNormalPrior<2>(context_, "prior", x0->Id(), prior_mean, prior_covariance);
  AddNormalDelta<2>(context_, "process", x0->Id(), x1->Id(), process_mean, process_covariance);
  AddNormalMeasurement<2>(context_, "measurement", x1->Id(), measurement_mean, measurement_covariance);

  auto middleware = std::make_shared<MockMiddleware>();
  std::vector<std::string> debug_messages;
  ON_CALL(*middleware->logger, Log(LogLevel::kDebug, testing::_))
      .WillByDefault([&debug_messages](LogLevel, std::string_view message) { debug_messages.emplace_back(message); });
  EXPECT_CALL(*middleware->logger, Log(LogLevel::kDebug, testing::_)).Times(testing::AnyNumber());

  ExtendedKalmanFilter::Configuration config;
  config.tracked_variable_names = {"x"};
  auto ekf = std::make_shared<ExtendedKalmanFilter>(middleware, config);

  ASSERT_TRUE(ekf->Execute(context_, ComputeBudget::Unlimited()));

  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("processing state chunk 0")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("applying process constraint 'process@")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("conditioning on [x@0] to predict [x@")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("predicted mean")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("applying measurement constraint 'measurement@")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("has residual")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("has innovation covariance diag")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("applied a correction of")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("posterior estimate at timestamp")));
  EXPECT_THAT(debug_messages, testing::Contains(testing::HasSubstr("published estimate at timestamp")));
}

}  // namespace
