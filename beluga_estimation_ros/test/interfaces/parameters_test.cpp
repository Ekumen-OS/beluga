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

#include <beluga_estimation_ros/interfaces/parameters.hpp>

namespace {

using beluga_estimation::ParameterType;
using beluga_estimation::ParameterValue;

class ParametersInterfaceTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(ParametersInterfaceTest, DeclareAndGetParameter) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto interface = beluga_estimation_ros::Parameters(node->get_node_parameters_interface());

  const auto value = interface.DeclareParameter("test_param", ParameterValue(1.23));

  EXPECT_EQ(value.GetType(), ParameterType::kDouble);
  EXPECT_DOUBLE_EQ(value.AsDouble(), 1.23);
  EXPECT_TRUE(interface.HasParameter("test_param"));
  EXPECT_DOUBLE_EQ(interface.GetParameter("test_param").AsDouble(), 1.23);
}

TEST_F(ParametersInterfaceTest, DeclareAndGetParameterWithTemplate) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto interface = beluga_estimation_ros::Parameters(node->get_node_parameters_interface());

  const auto value = interface.DeclareParameter<std::string>("test_param", "default_value");

  EXPECT_EQ(value, "default_value");
  EXPECT_TRUE(interface.HasParameter("test_param"));
  EXPECT_EQ(interface.GetParameter<std::string>("test_param"), "default_value");
}

TEST_F(ParametersInterfaceTest, DeclareWithOverride) {
  auto options = rclcpp::NodeOptions().parameter_overrides({{"test_param", 4.56}});
  auto node = std::make_shared<rclcpp::Node>("test_node", options);
  auto interface = beluga_estimation_ros::Parameters(node->get_node_parameters_interface());

  const auto value = interface.DeclareParameter("test_param", ParameterValue(1.23));

  EXPECT_DOUBLE_EQ(value.AsDouble(), 4.56);
}

TEST_F(ParametersInterfaceTest, GetParameterThrowsIfNotFound) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto interface = beluga_estimation_ros::Parameters(node->get_node_parameters_interface());

  EXPECT_THROW(
      static_cast<void>(interface.GetParameter("non_existent")), rclcpp::exceptions::ParameterNotDeclaredException);
}

TEST_F(ParametersInterfaceTest, NoDefault) {
  auto node = std::make_shared<rclcpp::Node>("no_default_param");
  auto interface = beluga_estimation_ros::Parameters(node->get_node_parameters_interface());

  const auto value = interface.DeclareParameter("no_default_param", ParameterType::kInteger);

  EXPECT_EQ(value.GetType(), ParameterType::kNotSet);

  EXPECT_TRUE(interface.HasParameter("no_default_param"));
  EXPECT_THROW(
      static_cast<void>(interface.GetParameter("no_default_param")),
      rclcpp::exceptions::ParameterUninitializedException);

  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("no_default_param", 42)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("no_default_param", "invalid_value")).successful);

  const auto new_value = interface.GetParameter("no_default_param");
  EXPECT_EQ(new_value.GetType(), ParameterType::kInteger);
  EXPECT_EQ(new_value.AsInt(), 42);
}

}  // namespace
