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
 * @brief Includes message registration and identification utilities.
 */

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation {

/// Unique identifier for a message type.
using MessageId = std::uint64_t;

/**
 * @brief Base class for all message types in Beluga.
 *
 * This class provides a common interface for accessing message metadata
 * and the underlying message data in a type-erased manner.
 */
class Message {
 public:
  virtual ~Message() = default;

  /**
   * @brief Returns the unique identifier for the message type.
   * @return The `MessageId` associated with this message's type.
   */
  virtual MessageId GetId() const = 0;

  /**
   * @brief Returns a pointer to the underlying native message object.
   *
   * The caller is responsible for casting this pointer to the correct type
   * based on the `MessageId`.
   */
  virtual const void* GetNativeHandle() const = 0;
};

/**
 * @brief Computes a 64-bit FNV-1a hash of a string view at compile time.
 *
 * @param str The string view to hash.
 * @return The 64-bit hash value.
 */
constexpr std::uint64_t HashStringView(std::string_view str) {
  std::uint64_t hash = 0xcbf29ce484222325ULL;
  for (char c : str) {
    hash ^= static_cast<std::uint64_t>(c);
    hash *= 0x100000001b3ULL;
  }
  return hash;
}

}  // namespace beluga_estimation
