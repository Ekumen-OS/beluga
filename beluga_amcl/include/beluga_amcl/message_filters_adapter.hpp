// Copyright 2025 Ekumen, Inc.
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

#ifndef BELUGA_AMCL_MESSAGE_FILTERS_ADAPTER_HPP
#define BELUGA_AMCL_MESSAGE_FILTERS_ADAPTER_HPP

#include <message_filters/subscriber.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Macro definition for message_filters version check (true if equal or higher than)
#define MESSAGE_FILTERS_VERSION_CHECK(major, minor, patch)                                  \
  ((MESSAGE_FILTERS_VERSION_MAJOR > (major)) ||                                             \
   (MESSAGE_FILTERS_VERSION_MAJOR == (major) && MESSAGE_FILTERS_VERSION_MINOR > (minor)) || \
   (MESSAGE_FILTERS_VERSION_MAJOR == (major) && MESSAGE_FILTERS_VERSION_MINOR == (minor) && \
    MESSAGE_FILTERS_VERSION_PATCH >= (patch)))

namespace beluga_amcl::compatibility::message_filters {

// mesagge_filters templetized adapter
#if MESSAGE_FILTERS_VERSION_CHECK(7, 2, 1)
template <typename MsgT>
using AdaptedSubscriber = ::message_filters::Subscriber<MsgT>;
#else
template <typename MsgT>
using AdaptedSubscriber = ::message_filters::Subscriber<MsgT, rclcpp_lifecycle::LifecycleNode>;
#endif

}  // namespace beluga_amcl::compatibility::message_filters

#endif
