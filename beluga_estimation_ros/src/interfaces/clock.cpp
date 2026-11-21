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
#include <beluga_estimation_ros/interfaces/clock.hpp>

namespace beluga_estimation_ros {

Clock::Clock(std::shared_ptr<rclcpp::Clock> clock) : clock_(std::move(clock)) {}

beluga_estimation::TimePoint Clock::Now() const {
  return beluga_estimation_ros::FromRos(clock_->now());
}

}  // namespace beluga_estimation_ros
