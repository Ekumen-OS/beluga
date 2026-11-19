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

#include <gmock/gmock.h>

#include <rclcpp/rclcpp.hpp>

#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation_ros/interfaces/middleware.hpp>

namespace {

class MiddlewareInterfaceTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

class TestProcess : public beluga_estimation::BayesianOperation {
 public:
  explicit TestProcess(std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware)
      : node_(beluga_estimation_ros::GetRosNodeInterface(std::move(middleware))) {
    auto params_interface = node_->get_node_parameters_interface();
    params_interface->declare_parameter("test_param", rclcpp::ParameterValue(1.23));
  }

  beluga_estimation::Result Execute(
      beluga_estimation::Context_<double>&,
      const beluga_estimation::ComputeBudget&) noexcept override {
    // Just a dummy implementation to test middleware integration.
    return beluga_estimation::Ok();
  };

 private:
  std::shared_ptr<beluga_estimation_ros::NodeInterface> node_;
};

TEST_F(MiddlewareInterfaceTest, InterfaceIntegration) {
  auto ros_node = std::make_shared<rclcpp::Node>("test_node");
  auto middleware = std::make_shared<beluga_estimation_ros::Middleware>(ros_node);
  auto node = std::make_shared<TestProcess>(middleware);

  EXPECT_TRUE(middleware->HasParameter("test_param"));
  EXPECT_DOUBLE_EQ(middleware->GetParameter<double>("test_param"), 1.23);
}

}  // namespace
