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
 * @brief Provides ROS 2 implementations for the Header message interface.
 */

#include <type_traits>

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>

#include <beluga_estimation/msg/header.hpp>
#include <beluga_estimation_ros/converters/fwd.hpp>
#include <beluga_estimation_ros/converters/types.hpp>

namespace beluga_estimation_ros::msg {

namespace detail {

/**
 * @brief Template implementation of the Header interface for ROS 2.
 * @tparam B The base Header interface to implement (Header or MutableHeader).
 */
template <typename Base>
class HeaderImpl : public Base {
 public:
  /// @brief The underlying ROS 2 message type.
  using RosType = std_msgs::msg::Header;
  /// @brief A shared pointer to the ROS 2 message, const-qualified if the interface is read-only.
  using Pointer = std::shared_ptr<std::conditional_t<Base::kIsReadOnly, const RosType, RosType>>;

  /// @brief Constructs a HeaderImpl from a shared pointer to a ROS 2 message.
  explicit HeaderImpl(Pointer message) : message_{std::move(message)} {}

  /// @brief Default constructor. Initializes the header with a new ROS 2 message instance.
  HeaderImpl() : HeaderImpl(std::make_shared<RosType>()) {}

  /// @brief Constructs a HeaderImpl by copying a ROS 2 message.
  explicit HeaderImpl(const RosType& message) : HeaderImpl(std::make_shared<RosType>(message)) {}

  /// @brief Constructs a HeaderImpl by moving a ROS 2 message.
  explicit HeaderImpl(RosType&& message) : HeaderImpl(std::make_shared<RosType>(std::move(message))) {}

  /// @brief Returns a pointer to the underlying ROS 2 message.
  const void* GetNativeHandle() const override { return this->message_.get(); }

  /// @brief Returns the timestamp from the ROS 2 message.
  beluga_estimation::TimePoint GetStamp() const override {
    return beluga_estimation_ros::FromRos(this->message_->stamp);
  }

  /// @brief Returns the frame ID from the ROS 2 message.
  std::string_view GetFrameId() const override { return this->message_->frame_id; }

 protected:
  /// @brief The shared ROS 2 message instance.
  Pointer message_;
};

}  // namespace detail

/**
 * @brief Read-only implementation of the Header interface for ROS 2.
 */
class Header final : public detail::HeaderImpl<beluga_estimation::msg::Header> {
 public:
  /// @brief Type alias for the base implementation.
  using Base = detail::HeaderImpl<beluga_estimation::msg::Header>;
  using Base::Base;
};

/**
 * @brief Mutable implementation of the Header interface for ROS 2.
 */
class MutableHeader final : public detail::HeaderImpl<beluga_estimation::msg::MutableHeader> {
 public:
  /// @brief Type alias for the base implementation.
  using Base = detail::HeaderImpl<beluga_estimation::msg::MutableHeader>;
  using Base::Base;

  /**
   * @brief Conversion operator to a read-only Header view.
   * @return A Header instance sharing the same underlying message.
   */
  operator beluga_estimation_ros::msg::Header() const { return beluga_estimation_ros::msg::Header(this->message_); }

  /// @brief Sets the timestamp in the ROS 2 message.
  void SetStamp(beluga_estimation::TimePoint stamp) override {
    this->message_->stamp = beluga_estimation_ros::ToRos(stamp);
  }

  /// @brief Sets the frame ID in the ROS 2 message.
  void SetFrameId(std::string_view frame_id) override { this->message_->frame_id = frame_id; }
};

}  // namespace beluga_estimation_ros::msg

namespace beluga_estimation_ros {

/**
 * @brief Specialization of TypeConverter for beluga_estimation::msg::Header.
 */
template <>
struct TypeConverter<beluga_estimation::msg::Header> {
  /// @brief Converts a Beluga Header to its ROS 2 representation.
  static std_msgs::msg::Header ToRos(const beluga_estimation::msg::Header& message) {
    return *reinterpret_cast<const std_msgs::msg::Header*>(message.GetNativeHandle());
  }

  /// @brief Conversion from ROS 2 to Beluga Header is not supported via this interface.
  static beluga_estimation::msg::Header FromRos(const std_msgs::msg::Header&) = delete;
};

/// @brief Specialization of TypeConverter for std_msgs::msg::Header.
template <>
struct TypeConverter<std_msgs::msg::Header> : TypeConverter<beluga_estimation::msg::Header> {};

}  // namespace beluga_estimation_ros
