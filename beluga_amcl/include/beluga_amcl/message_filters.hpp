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

#ifndef BELUGA_AMCL_MESSAGE_FILTERS_HPP
#define BELUGA_AMCL_MESSAGE_FILTERS_HPP

#include <message_filters/subscriber.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/**
 * \file
 * \brief Compatibility layer for \p message_filters.
 */

/// Check \p message_filters version.
/**
 * Expression is true when the \p message_filters version is greater than or equal to the specified version.
 *
 * This macro shall be removed as soon as \p message_filters starts providing a suitable version header.
 */
#define BELUGA_AMCL_MESSAGE_FILTERS_VERSION_GTE(major, minor, patch)                                                \
  ((BELUGA_AMCL_MESSAGE_FILTERS_VERSION_MAJOR > (major)) ||                                                         \
   (BELUGA_AMCL_MESSAGE_FILTERS_VERSION_MAJOR == (major) && BELUGA_AMCL_MESSAGE_FILTERS_VERSION_MINOR > (minor)) || \
   (BELUGA_AMCL_MESSAGE_FILTERS_VERSION_MAJOR == (major) && BELUGA_AMCL_MESSAGE_FILTERS_VERSION_MINOR == (minor) && \
    BELUGA_AMCL_MESSAGE_FILTERS_VERSION_PATCH >= (patch)))

namespace beluga_amcl::message_filters {

/// \p message_filters::Subscriber specialization
/**
 * Standardizes template type to its most recent form.
 */
#if BELUGA_AMCL_MESSAGE_FILTERS_VERSION_GTE(7, 2, 1)
template <typename MsgT>
using Subscriber = ::message_filters::Subscriber<MsgT>;
#else
template <typename MsgT>
using Subscriber = ::message_filters::Subscriber<MsgT, rclcpp_lifecycle::LifecycleNode>;
#endif

}  // namespace beluga_amcl::message_filters

#endif  // BELUGA_AMCL_MESSAGE_FILTERS_HPP
