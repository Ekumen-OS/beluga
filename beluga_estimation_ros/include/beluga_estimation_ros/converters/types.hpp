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

#include <builtin_interfaces/msg/time.hpp>

#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation_ros/converters/fwd.hpp>

namespace beluga_estimation_ros {

template <>
struct TypeConverter<beluga_estimation::TimePoint> {
  static builtin_interfaces::msg::Time ToRos(const beluga_estimation::TimePoint& message) {
    builtin_interfaces::msg::Time ros_message;
    ros_message.sec = static_cast<std::int32_t>(message.time_since_epoch().count() / 1000000000LL);
    ros_message.nanosec = static_cast<std::uint32_t>(message.time_since_epoch().count() % 1000000000LL);
    return ros_message;
  }

  static beluga_estimation::TimePoint FromRos(const builtin_interfaces::msg::Time& message) {
    return beluga_estimation::TimePoint{std::chrono::seconds(message.sec) + std::chrono::nanoseconds(message.nanosec)};
  }
};

template <>
struct TypeConverter<builtin_interfaces::msg::Time> : TypeConverter<beluga_estimation::TimePoint> {};

}  // namespace beluga_estimation_ros
