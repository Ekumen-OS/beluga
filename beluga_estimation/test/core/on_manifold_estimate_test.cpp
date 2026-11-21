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

#include <cstddef>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <range/v3/range/primitives.hpp>

#include <beluga_estimation/common/random.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/distributions/generic_tangent_distribution.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/manifolds/euclidean_manifold.hpp>
#include <beluga_estimation/manifolds/product_manifold.hpp>
#include <beluga_estimation/manifolds/se2_manifold.hpp>
#include <beluga_estimation/mocks/mock_variable.hpp>

namespace beluga_estimation {
namespace {

/// @brief Creates a NormalDistribution_ with the given dimension, zero mean,
///        and identity covariance.
std::shared_ptr<NormalDistribution_<double, Eigen::Dynamic>> MakeIdentityNormal(std::size_t dim) {
  return std::make_shared<NormalDistribution_<double, Eigen::Dynamic>>(
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dim)),
      Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(dim), static_cast<Eigen::Index>(dim)));
}

/// @brief Creates a mock variable with the given identifier and manifold
///        expectation.
template <class ManifoldT>
std::shared_ptr<MockVariable_<double>> MakeMockVariable(const Identifier& id, std::shared_ptr<ManifoldT> manifold) {
  auto var = std::make_shared<MockVariable_<double>>(id);
  EXPECT_CALL(*var, Manifold()).WillRepeatedly(testing::Return(manifold));
  return var;
}

TEST(OnManifoldEstimate, ConstructionAndAccess) {
  // 1. Create manifolds.
  auto se2 = SE2Manifold_<double>::Instance();
  auto e2 = EuclideanManifold_<double, 2>::Instance();

  // 2. Instantiate variables (using those manifolds).
  auto mock_pose = MakeMockVariable(Identifier{.name = "pose", .timestamp = TimePoint{}}, se2);
  auto mock_vel = MakeMockVariable(Identifier{.name = "velocity", .timestamp = TimePoint{}}, e2);

  std::vector<std::shared_ptr<const Variable_<double>>> variables;
  variables.push_back(mock_pose);
  variables.push_back(mock_vel);

  // 3. Construct the product manifold from the variables' manifolds.
  ProductManifoldBuilder_<double> builder;
  builder.Add(se2);
  builder.Add(e2);
  auto manifold = builder.Build();

  // 4. Construct the distribution.
  auto normal = MakeIdentityNormal(5);
  Eigen::VectorXd origin_data(6);
  origin_data << 1.0, 0.0, 10.0, 20.0, 100.0, 200.0;
  auto distribution = std::make_shared<GenericTangentDistribution_<double>>(manifold, normal, std::move(origin_data));

  // 5. Construct the estimate.
  OnManifoldEstimate_<double> estimate(std::move(variables), distribution);

  auto vars = estimate.GetVariables();
  EXPECT_EQ(ranges::size(vars), 2u);
  auto it = ranges::begin(vars);
  EXPECT_EQ((*it)->Id().name, "pose");
  ++it;
  EXPECT_EQ((*it)->Id().name, "velocity");

  // Manifold accessible through the tangent distribution.
  auto est_manifold = estimate.GetDistribution()->GetManifold();
  EXPECT_EQ(est_manifold->AmbientSize(), 6u);
  EXPECT_EQ(est_manifold->TangentSize(), 5u);

  EXPECT_NE(estimate.GetDistribution(), nullptr);
}

TEST(OnManifoldEstimate, MarginalizeToSingleVariable) {
  // 1. Create manifolds.
  auto e3 = EuclideanManifold_<double, 3>::Instance();
  auto e2 = EuclideanManifold_<double, 2>::Instance();

  // 2. Instantiate variables.
  auto mock_a = MakeMockVariable(Identifier{.name = "a", .timestamp = TimePoint{}}, e3);
  auto mock_b = MakeMockVariable(Identifier{.name = "b", .timestamp = TimePoint{}}, e2);

  std::vector<std::shared_ptr<const Variable_<double>>> variables;
  variables.push_back(mock_a);
  variables.push_back(mock_b);

  // 3. Construct the product manifold from the variables' manifolds.
  ProductManifoldBuilder_<double> builder;
  builder.Add(e3);
  builder.Add(e2);
  auto manifold = builder.Build();

  // 4. Construct the distribution.
  auto normal = MakeIdentityNormal(5);
  Eigen::VectorXd origin_data = Eigen::VectorXd::Zero(5);
  auto distribution = std::make_shared<GenericTangentDistribution_<double>>(manifold, normal, std::move(origin_data));

  // 5. Construct the estimate.
  OnManifoldEstimate_<double> estimate(std::move(variables), distribution);

  // Marginalize to first variable (a) only.
  const Identifier first_only{.name = "a", .timestamp = TimePoint{}};
  auto result = estimate.Marginalize(first_only);
  ASSERT_NE(result, nullptr);

  auto result_vars = result->GetVariables();
  ASSERT_EQ(ranges::size(result_vars), 1u);
  EXPECT_EQ((*ranges::begin(result_vars))->Id().name, "a");

  // Access manifold through the result's tangent distribution.
  auto result_on_manifold = std::dynamic_pointer_cast<const OnManifoldEstimate_<double>>(result);
  ASSERT_NE(result_on_manifold, nullptr);
  auto result_manifold = result_on_manifold->GetDistribution()->GetManifold();
  EXPECT_EQ(result_manifold->AmbientSize(), 3u);  // Euclidean3
  EXPECT_EQ(result_manifold->TangentSize(), 3u);
}

TEST(OnManifoldEstimate, MarginalizeToMultiVariableSubset) {
  // 1. Create manifolds.
  auto e2 = EuclideanManifold_<double, 2>::Instance();
  auto e3 = EuclideanManifold_<double, 3>::Instance();
  auto e1 = EuclideanManifold_<double, 1>::Instance();

  // 2. Instantiate variables.
  auto mock_a = MakeMockVariable(Identifier{.name = "a", .timestamp = TimePoint{}}, e2);
  auto mock_b = MakeMockVariable(Identifier{.name = "b", .timestamp = TimePoint{}}, e3);
  auto mock_c = MakeMockVariable(Identifier{.name = "c", .timestamp = TimePoint{}}, e1);

  std::vector<std::shared_ptr<const Variable_<double>>> variables;
  variables.push_back(mock_a);
  variables.push_back(mock_b);
  variables.push_back(mock_c);

  // 3. Construct the product manifold from the variables' manifolds.
  ProductManifoldBuilder_<double> builder;
  builder.Add(e2);
  builder.Add(e3);
  builder.Add(e1);
  auto manifold = builder.Build();

  // 4. Construct the distribution.
  auto normal = MakeIdentityNormal(6);
  Eigen::VectorXd origin_data = Eigen::VectorXd::Zero(6);
  auto distribution = std::make_shared<GenericTangentDistribution_<double>>(manifold, normal, std::move(origin_data));

  // 5. Construct the estimate.
  OnManifoldEstimate_<double> estimate(std::move(variables), distribution);

  // Marginalize to {a, c} (skip b).
  std::vector<Identifier> subset = {
      Identifier{.name = "a", .timestamp = TimePoint{}}, Identifier{.name = "c", .timestamp = TimePoint{}}};
  auto result = estimate.Marginalize(subset);
  ASSERT_NE(result, nullptr);

  auto result_vars = result->GetVariables();
  ASSERT_EQ(ranges::size(result_vars), 2u);
  auto it = ranges::begin(result_vars);
  EXPECT_EQ((*it)->Id().name, "a");
  ++it;
  EXPECT_EQ((*it)->Id().name, "c");

  // Access manifold through the result's tangent distribution.
  auto result_on_manifold = std::dynamic_pointer_cast<const OnManifoldEstimate_<double>>(result);
  ASSERT_NE(result_on_manifold, nullptr);
  auto result_manifold = result_on_manifold->GetDistribution()->GetManifold();
  EXPECT_EQ(result_manifold->AmbientSize(), 3u);  // e2(2) + e1(1)
  EXPECT_EQ(result_manifold->TangentSize(), 3u);  // e2(2) + e1(1)
}

TEST(OnManifoldEstimate, MarginalizeUnknownVariableReturnsNull) {
  // 1. Create manifolds.
  auto e2 = EuclideanManifold_<double, 2>::Instance();

  // 2. Instantiate variables.
  auto mock_x = MakeMockVariable(Identifier{.name = "x", .timestamp = TimePoint{}}, e2);

  std::vector<std::shared_ptr<const Variable_<double>>> variables;
  variables.push_back(mock_x);

  // 3. Construct the product manifold from the variables' manifolds.
  ProductManifoldBuilder_<double> builder;
  builder.Add(e2);
  auto manifold = builder.Build();

  // 4. Construct the distribution.
  auto normal = MakeIdentityNormal(2);
  auto distribution = std::make_shared<GenericTangentDistribution_<double>>(manifold, normal, Eigen::VectorXd::Zero(2));

  // 5. Construct the estimate.
  OnManifoldEstimate_<double> estimate(std::move(variables), distribution);

  std::vector<Identifier> unknown = {Identifier{.name = "unknown", .timestamp = TimePoint{}}};
  EXPECT_EQ(estimate.Marginalize(unknown), nullptr);
}

TEST(OnManifoldEstimate, MarginalizeEmptyReturnsNull) {
  // 1. Create manifolds.
  auto e2 = EuclideanManifold_<double, 2>::Instance();

  // 2. Instantiate variables.
  auto mock_x = MakeMockVariable(Identifier{.name = "x", .timestamp = TimePoint{}}, e2);

  std::vector<std::shared_ptr<const Variable_<double>>> variables;
  variables.push_back(mock_x);

  // 3. Construct the product manifold from the variables' manifolds.
  ProductManifoldBuilder_<double> builder;
  builder.Add(e2);
  auto manifold = builder.Build();

  // 4. Construct the distribution.
  auto normal = MakeIdentityNormal(2);
  auto distribution = std::make_shared<GenericTangentDistribution_<double>>(manifold, normal, Eigen::VectorXd::Zero(2));

  // 5. Construct the estimate.
  OnManifoldEstimate_<double> estimate(std::move(variables), distribution);

  EXPECT_EQ(estimate.Marginalize(std_future::span<const beluga_estimation::Identifier>{}), nullptr);
}

TEST(OnManifoldEstimate, MarginalizeSameVariableSucceeds) {
  // 1. Create manifolds.
  auto e2 = EuclideanManifold_<double, 2>::Instance();

  // 2. Instantiate variables.
  auto mock_x = MakeMockVariable(Identifier{.name = "x", .timestamp = TimePoint{}}, e2);

  std::vector<std::shared_ptr<const Variable_<double>>> variables;
  variables.push_back(mock_x);

  // 3. Construct the product manifold from the variables' manifolds.
  ProductManifoldBuilder_<double> builder;
  builder.Add(e2);
  auto manifold = builder.Build();

  // 4. Construct the distribution.
  auto normal = MakeIdentityNormal(2);
  auto distribution = std::make_shared<GenericTangentDistribution_<double>>(manifold, normal, Eigen::VectorXd::Zero(2));

  // 5. Construct the estimate.
  OnManifoldEstimate_<double> estimate(std::move(variables), distribution);

  // Marginalize to same variable (which is the only one — full set).
  const Identifier same{.name = "x", .timestamp = TimePoint{}};
  auto result = estimate.Marginalize(same);
  ASSERT_NE(result, nullptr);
}

}  // namespace
}  // namespace beluga_estimation
