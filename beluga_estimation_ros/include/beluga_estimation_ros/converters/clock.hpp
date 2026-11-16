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

/**
 * @file
 * @brief Provides conversion between rclcpp clock types and beluga time types.
 */

#include <rclcpp/time.hpp>

#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation_ros/converters/fwd.hpp>

namespace beluga_estimation_ros {

template <>
struct TypeConverter<rclcpp::Time> {
  static_assert(
      std::ratio_less_equal_v<beluga_estimation::TimePoint::duration::period, std::nano>,
      "TimePoint duration must have at least nanosecond precision for lossless rclcpp::Time conversion");

  static beluga_estimation::TimePoint FromRos(const rclcpp::Time& time) {
    return beluga_estimation::TimePoint(std::chrono::nanoseconds(time.nanoseconds()));
  }
};

}  // namespace beluga_estimation_ros
