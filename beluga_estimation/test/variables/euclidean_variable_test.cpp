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

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/distributions/generic_tangent_distribution.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/mocks/mock_estimate.hpp>
#include <beluga_estimation/variables/euclidean_variable.hpp>

namespace {

using ::testing::Return;

// Builds a tangent distribution over a Euclidean manifold with the given mean
// and identity covariance. For Euclidean, the origin is implicitly zero.
std::shared_ptr<beluga_estimation::GenericTangentDistribution_<double>> MakeEuclideanTangentDistribution(
    std::shared_ptr<const beluga_estimation::Manifold_<double>> manifold,
    beluga_estimation::VectorXd mean) {
  auto tangent_dist = std::make_shared<beluga_estimation::NormalDistribution_<double>>(
      mean, beluga_estimation::MatrixXd::Identity(mean.size(), mean.size()));
  return std::make_shared<beluga_estimation::GenericTangentDistribution_<double>>(manifold, std::move(tangent_dist));
}

// Builds an OnManifoldEstimate for a single variable with the given tangent
// distribution.
std::shared_ptr<beluga_estimation::OnManifoldEstimate_<double>> MakeEstimate(
    std::shared_ptr<const beluga_estimation::Variable_<double>> variable,
    std::shared_ptr<beluga_estimation::GenericTangentDistribution_<double>> tangent) {
  return std::make_shared<beluga_estimation::OnManifoldEstimate_<double>>(
      std::vector<std::shared_ptr<const beluga_estimation::Variable_<double>>>{std::move(variable)},
      std::move(tangent));
}

TEST(EuclideanVariableTest, ManifoldDimension) {
  // Both the compile-time-tagged and the runtime-dimension constructors report
  // their dimension through the variable.
  beluga_estimation::Identifier id{.name = "test_vector"};
  beluga_estimation::EuclideanVariable fixed_var(id, beluga_estimation::NDim<3>{});
  EXPECT_EQ(fixed_var.Id().name, std::string("test_vector"));
  EXPECT_EQ(fixed_var.Size(), 3);
  EXPECT_EQ(fixed_var.Manifold()->AmbientSize(), 3);
  EXPECT_EQ(fixed_var.Manifold()->TangentSize(), 3);

  beluga_estimation::EuclideanVariable dynamic_var(id, 4);
  EXPECT_EQ(dynamic_var.Id().name, std::string("test_vector"));
  EXPECT_EQ(dynamic_var.Size(), 4);
  EXPECT_EQ(dynamic_var.Manifold()->AmbientSize(), 4);
  EXPECT_EQ(dynamic_var.Manifold()->TangentSize(), 4);
}

TEST(EuclideanVariableTest, CompileTimeDimensionBindsManifoldSingleton) {
  // The NDim constructor must bind the statically sized singleton rather than
  // allocate a manifold, since probes construct one variable per message.
  beluga_estimation::Identifier id{.name = "test_vector"};
  beluga_estimation::EuclideanVariable first(id, beluga_estimation::NDim<3>{});
  beluga_estimation::EuclideanVariable second(id, beluga_estimation::NDim<3>{});
  EXPECT_EQ(first.Manifold(), (beluga_estimation::EuclideanManifold_<double, 3>::Instance()));
  EXPECT_EQ(second.Manifold(), (beluga_estimation::EuclideanManifold_<double, 3>::Instance()));
  EXPECT_EQ(first.Manifold(), second.Manifold());
}

TEST(EuclideanVariableTest, RuntimeDimensionAllocatesOwnManifold) {
  beluga_estimation::Identifier id{.name = "test_vector"};
  beluga_estimation::EuclideanVariable first(id, 3);
  beluga_estimation::EuclideanVariable second(id, 3);
  EXPECT_NE(first.Manifold(), second.Manifold());
}

TEST(EuclideanVariableTest, DynamicVariableGetDistribution) {
  beluga_estimation::Identifier id{.name = "test_vector"};
  auto var = std::make_shared<beluga_estimation::EuclideanVariable>(id, 2);

  beluga_estimation::VectorXd mean(2);
  mean << 4.0, 5.0;
  auto estimate = MakeEstimate(var, MakeEuclideanTangentDistribution(var->Manifold(), mean));

  auto result = var->GetDistribution(*estimate);
  ASSERT_NE(result, nullptr);

  // Check the result is an EuclideanDistribution of size 2.
  std::vector<double> result_mean(2);
  result->ComputeMean(result_mean);
  EXPECT_NEAR(result_mean[0], 4.0, 1e-6);
  EXPECT_NEAR(result_mean[1], 5.0, 1e-6);
}

TEST(EuclideanVariableTest, GetDistributionNullptrOnNoMarginalize) {
  beluga_estimation::Identifier id{.name = "test_vector"};
  beluga_estimation::EuclideanVariable var(id, beluga_estimation::NDim<3>{});

  auto mock_est = std::make_shared<beluga_estimation::MockEstimate_<double>>();
  EXPECT_CALL(*mock_est, DoMarginalize(testing::_)).WillOnce(Return(nullptr));

  auto result = var.GetDistribution(*mock_est);
  EXPECT_EQ(result, nullptr);
}

TEST(EuclideanVariableTest, GetDistributionReturnsEuclidean) {
  beluga_estimation::Identifier id{.name = "test_vector"};
  auto var = std::make_shared<beluga_estimation::EuclideanVariable>(id, beluga_estimation::NDim<3>{});

  beluga_estimation::Vector3d mean;
  mean << 1.0, 2.0, 3.0;
  auto estimate = MakeEstimate(var, MakeEuclideanTangentDistribution(var->Manifold(), mean));

  auto result = var->GetDistribution(*estimate);
  ASSERT_NE(result, nullptr);

  // Check the result is an EuclideanDistribution of size 3.
  std::vector<double> result_mean(3);
  result->ComputeMean(result_mean);
  EXPECT_NEAR(result_mean[0], 1.0, 1e-6);
  EXPECT_NEAR(result_mean[1], 2.0, 1e-6);
  EXPECT_NEAR(result_mean[2], 3.0, 1e-6);
}

}  // namespace
