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
#include <rclcpp/rate.hpp>

#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/middleware/rate_interface.hpp>

namespace beluga_estimation_ros {

/**
 * @brief ROS 2 implementation of the Beluga rate interface.
 *
 * Wraps rclcpp::Rate to control loop execution frequency, paced by the
 * provided clock.
 */
class Rate : public beluga_estimation::RateInterface {
 public:
  /**
   * @brief Constructs a Rate instance.
   * @param frequency The desired loop frequency in Hz.
   * @param clock The rclcpp clock that drives both the rate and budget checks.
   */
  Rate(double frequency, std::shared_ptr<rclcpp::Clock> clock);

  /// \copydoc beluga_estimation::RateInterface::Sleep
  beluga_estimation::Result Sleep(const beluga_estimation::ComputeBudget& budget) override;

  /// \copydoc beluga_estimation::RateInterface::Reset
  void Reset() override;

 private:
  std::shared_ptr<rclcpp::Clock> clock_;
  rclcpp::Rate rate_;
};

}  // namespace beluga_estimation_ros
