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

#pragma once

#include <memory>

#include <rclcpp/clock.hpp>

#include <beluga_estimation/middleware/clock_interface.hpp>

namespace beluga_estimation_ros {

/**
 * @brief ROS 2 implementation of the Beluga clock interface.
 *
 * Wraps an rclcpp::Clock to expose the current time through the
 * middleware-agnostic @ref beluga_estimation::ClockInterface.
 */
class Clock : public beluga_estimation::ClockInterface {
 public:
  /**
   * @brief Constructs a Clock instance.
   * @param clock The rclcpp clock to wrap.
   */
  explicit Clock(std::shared_ptr<rclcpp::Clock> clock);

  /// \copydoc beluga_estimation::ClockInterface::Now
  beluga_estimation::TimePoint Now() const override;

 private:
  std::shared_ptr<rclcpp::Clock> clock_;
};

}  // namespace beluga_estimation_ros
