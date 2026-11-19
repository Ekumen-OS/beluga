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

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation_ros/interfaces/middleware.hpp>

namespace {

class ClockRateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    ros_node_ = std::make_shared<rclcpp::Node>("clock_rate_test_node");
    middleware_ = std::make_shared<beluga_estimation_ros::Middleware>(ros_node_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<beluga_estimation_ros::Middleware> middleware_;
};

TEST_F(ClockRateTest, GetClockInterfaceReturnsNonNull) {
  EXPECT_NE(middleware_->GetClockInterface(), nullptr);
}

TEST_F(ClockRateTest, ClockNowReturnsPositiveTime) {
  auto now = middleware_->GetClockInterface()->Now();
  EXPECT_GT(now, beluga_estimation::TimePoint{});
}

TEST_F(ClockRateTest, CreateRateReturnsNonNull) {
  EXPECT_NE(middleware_->CreateRate(10.0), nullptr);
}

TEST_F(ClockRateTest, RateSleepReturnsTimeoutForExpiredBudget) {
  auto rate = middleware_->CreateRate(10.0);
  auto budget = beluga_estimation::ComputeBudget(beluga_estimation::TimePoint{}, {});
  auto result = rate->Sleep(budget);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kTimeout);
}

TEST_F(ClockRateTest, RateSleepReturnsInterruptedWhenStopped) {
  auto rate = middleware_->CreateRate(10.0);
  beluga_estimation::std_future::stop_source stop;
  stop.request_stop();
  auto budget = beluga_estimation::ComputeBudget(std::nullopt, stop.get_token());
  auto result = rate->Sleep(budget);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kInterrupted);
}

TEST_F(ClockRateTest, RateResetDoesNotCorruptState) {
  auto rate = middleware_->CreateRate(10.0);
  rate->Reset();
  auto budget = beluga_estimation::ComputeBudget(beluga_estimation::TimePoint{}, {});
  auto result = rate->Sleep(budget);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error(), beluga_estimation::Error::kTimeout);
}

}  // namespace
