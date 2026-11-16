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
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <sophus/se2.hpp>

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/distributions/generic_tangent_distribution.hpp>
#include <beluga_estimation/distributions/normal_distribution.hpp>
#include <beluga_estimation/mocks/mock_estimate.hpp>
#include <beluga_estimation/variables/se2_variable.hpp>

using beluga_estimation::Identifier;
using beluga_estimation::SE2Variable;
using ::testing::Return;

class SE2VariableTest : public ::testing::Test {
 protected:
  void SetUp() override {
    id_ = Identifier{"test/pose", std::chrono::system_clock::time_point(std::chrono::nanoseconds(0))};
    variable_ = std::make_unique<SE2Variable>(id_);
  }

  Identifier id_;
  std::unique_ptr<SE2Variable> variable_;
};

TEST_F(SE2VariableTest, Manifold) {
  ASSERT_NE(variable_->Manifold(), nullptr);
  EXPECT_EQ(variable_->Manifold()->AmbientSize(), 4);
  EXPECT_EQ(variable_->Manifold()->TangentSize(), 3);
}

TEST_F(SE2VariableTest, Id) {
  EXPECT_EQ(variable_->Id().name, "test/pose");
}

TEST_F(SE2VariableTest, GetDistributionNullptrOnNoMarginalize) {
  // An estimate that does not support marginalization should return nullptr.
  auto mock_est = std::make_shared<beluga_estimation::MockEstimate_<double>>();
  EXPECT_CALL(*mock_est, DoMarginalize(testing::_)).WillOnce(Return(nullptr));

  const auto& var_base = static_cast<const beluga_estimation::Variable_<double>&>(*variable_);
  auto result = var_base.GetDistribution(*mock_est);
  EXPECT_EQ(result, nullptr);
}

TEST_F(SE2VariableTest, GetDistributionReturnsSE2Tangent) {
  // Build an OnManifoldEstimate with an SE2 variable.
  // The SE2 manifold has ambient size 4, tangent size 3.
  auto se2_manifold = beluga_estimation::SE2Manifold_<double>::Instance();
  auto var_shared = std::make_shared<SE2Variable>(id_);

  // Create a 3D normal distribution for the tangent space.
  Eigen::Matrix<double, 3, 1> mean;
  mean << 0.1, 0.0, 0.0;  // small x translation
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov;
  cov.setIdentity();
  auto normal_dist = std::make_shared<beluga_estimation::NormalDistribution_<double, 3>>(mean, cov);

  // Create a GenericTangentDistribution with an identity SE2 origin.
  // SE2 data layout: [cos(0)=1, sin(0)=0, x=0, y=0]
  Eigen::VectorXd origin(4);
  origin << 1.0, 0.0, 0.0, 0.0;
  auto tangent =
      std::make_shared<beluga_estimation::GenericTangentDistribution_<double>>(se2_manifold, normal_dist, origin);

  auto estimate = std::make_shared<beluga_estimation::OnManifoldEstimate_<double>>(
      std::vector<std::shared_ptr<const beluga_estimation::Variable_<double>>>{var_shared}, tangent);

  // Call the typed getter directly.
  auto result = var_shared->GetDistribution(*estimate);
  ASSERT_NE(result, nullptr);

  // Verify the result is an SE2TangentDistribution by checking it has
  // the expected tangent size (3 for SE2).
  std::vector<double> cov_data(9, 0.0);
  beluga_estimation::std_future::mdspan<double, 2> cov_span(cov_data.data(), 3, 3);
  result->ComputeCovariance(cov_span);
  // A NormalDistribution with 3-sigma=1 has identity covariance.
  EXPECT_NEAR(cov_data[0], 1.0, 1e-6);
  EXPECT_NEAR(cov_data[4], 1.0, 1e-6);
  EXPECT_NEAR(cov_data[8], 1.0, 1e-6);
}
