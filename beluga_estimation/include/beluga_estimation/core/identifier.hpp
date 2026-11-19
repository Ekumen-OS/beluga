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

#include <chrono>
#include <functional>
#include <string>
#include <tuple>

#include <fmt/format.h>

#include <beluga_estimation/common/hash.hpp>
#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation {

/**
 * @brief A class to uniquely identify elements in the belief propagation framework.
 *
 * An identifier is a composite key that consists of a name and a timestamp.
 *
 * The name is a string that represents the semantic meaning of some information.
 * Names may be hierarchical using forward-slash delimiters to represent nested
 * scopes or qualifiers, ordered from general to specific (left to right).
 *
 * Examples:
 * - Simple name: "pose"
 * - Source-qualified: "lidar/pose", "odom/velocity"
 * - Fully qualified: "robot/lidar/pose", "robot/odom/velocity"
 *
 * The timestamp is a time point that represents the time at which the information
 * is relevant.
 *
 * Identifiers are used to establish connections and dependencies between different
 * elements in the belief propagation framework. For example, a measurement from a
 * lidar sensor at a certain time can be associated with a specific state of the
 * robot at that same time using "lidar/measurement@t" and "robot/pose@t".
 */
struct Identifier {
  std::string name;
  TimePoint timestamp;

  bool operator==(const Identifier& other) const {
    return std::tie(timestamp, name) == std::tie(other.timestamp, other.name);
  }

  bool operator!=(const Identifier& other) const { return !(*this == other); }
};

inline std::ostream& operator<<(std::ostream& os, const Identifier& id) {
  return os << id.name << "@" << id.timestamp.time_since_epoch().count();
}

}  // namespace beluga_estimation

/// @brief Formats a `beluga_estimation::Identifier` as `name@timestamp`.
template <>
struct fmt::formatter<beluga_estimation::Identifier> : fmt::formatter<std::string_view> {
  /**
   * @brief Renders @p id into @p context.
   * @param id Identifier to render.
   * @param context Destination format context.
   * @return Iterator past the last written character.
   */
  template <typename FormatContext>
  auto format(const beluga_estimation::Identifier& id, FormatContext& context) const {
    return fmt::formatter<std::string_view>::format(
        fmt::format("{}@{}", id.name, id.timestamp.time_since_epoch().count()), context);
  }
};

namespace std {
template <>
struct hash<beluga_estimation::Identifier> {
  std::size_t operator()(const beluga_estimation::Identifier& id) const {
    std::size_t seed = 0;
    beluga_estimation::HashCombine(seed, id.name);
    beluga_estimation::HashCombine(seed, id.timestamp.time_since_epoch().count());
    return seed;
  }
};
}  // namespace std
