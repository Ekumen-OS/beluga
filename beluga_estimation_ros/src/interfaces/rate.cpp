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

#include <beluga_estimation_ros/converters/clock.hpp>
#include <beluga_estimation_ros/interfaces/rate.hpp>

namespace beluga_estimation_ros {

Rate::Rate(double frequency, std::shared_ptr<rclcpp::Clock> clock)
    : clock_(clock), rate_(frequency, std::move(clock)) {}

void Rate::Reset() {
  rate_.reset();
}

beluga_estimation::Result Rate::Sleep(const beluga_estimation::ComputeBudget& budget) {
  auto result = budget.Check(beluga_estimation_ros::FromRos(clock_->now()));
  if (!result) {
    return result;
  }
  rate_.sleep();
  return budget.Check(beluga_estimation_ros::FromRos(clock_->now()));
}

}  // namespace beluga_estimation_ros
