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
 * @brief Defines the Header message interface.
 */

#include <string_view>

#include <beluga_estimation/common/message.hpp>

namespace beluga_estimation::msg {

/**
 * @brief Interface for a message header containing metadata.
 *
 * This class provides read-only access to common message metadata like
 * timestamps and coordinate frame identifiers.
 */
class Header : public Message {
 public:
  /// @brief Type alias for the base message type.
  using BaseType = Header;

  /// @brief Indicates if this class is a read-only interface.
  static constexpr bool kIsReadOnly = true;

  /**
   * @brief Returns the static unique identifier for the Header message type.
   * @return The `MessageId` for Header.
   */
  static constexpr MessageId GetStaticId() { return HashStringView("beluga_estimation::msg::Header"); }

  /**
   * @brief Returns the unique identifier for the message type.
   * @return The `MessageId` associated with this message's type.
   */
  MessageId GetId() const final { return GetStaticId(); }

  /**
   * @brief Returns the timestamp associated with the message.
   * @return The `TimePoint` when the message was created or captured.
   */
  virtual beluga_estimation::TimePoint GetStamp() const = 0;

  /**
   * @brief Returns the coordinate frame identifier.
   * @return A `std::string_view` of the frame ID.
   */
  virtual std::string_view GetFrameId() const = 0;
};

/**
 * @brief Interface for a mutable message header.
 *
 * This class extends `Header` to provide setter methods for modifying
 * header metadata.
 */
class MutableHeader : public Header {
 public:
  /// @brief Indicates if this class is a read-only interface. False for mutable interfaces.
  static constexpr bool kIsReadOnly = false;

  /**
   * @brief Sets the timestamp for the message.
   * @param stamp The new `TimePoint` to set.
   */
  virtual void SetStamp(beluga_estimation::TimePoint stamp) = 0;

  /**
   * @brief Sets the coordinate frame identifier.
   * @param frame_id The new frame ID to set.
   */
  virtual void SetFrameId(std::string_view frame_id) = 0;
};

}  // namespace beluga_estimation::msg
