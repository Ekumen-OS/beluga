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
 * @brief Utilities for boolean masks used as bitsets.
 *
 * Beluga uses `std::vector<bool>` as its bitset type: it is what boolean
 * parameter masks are declared as, and what set membership checks operate on.
 * The functions below give that type a single vocabulary for clearing,
 * testing, updating, and enumerating bits.
 */

#include <algorithm>
#include <cstddef>
#include <vector>

#include <beluga_estimation/common/assert.hpp>

namespace beluga_estimation {

/// @brief Resets every bit in a bitset.
/// @param bits The bitset to clear.
inline void ClearBitSet(std::vector<bool>& bits) {
  std::fill(bits.begin(), bits.end(), false);
}

/// @brief Reports whether at least one bit is set.
/// @param bits The bitset to query.
/// @return `true` if any bit is set.
inline bool AnyBitSet(const std::vector<bool>& bits) {
  return std::find(bits.begin(), bits.end(), true) != bits.end();
}

/// @brief Returns the number of set bits.
/// @param bits The bitset to query.
/// @return The count of set bits.
inline std::size_t CountBitSet(const std::vector<bool>& bits) {
  return static_cast<std::size_t>(std::count(bits.begin(), bits.end(), true));
}

/// @brief Reports whether every bit is set.
/// @param bits The bitset to query.
/// @return `true` if all bits are set; `true` for an empty bitset.
inline bool AllBitSet(const std::vector<bool>& bits) {
  return std::find(bits.begin(), bits.end(), false) == bits.end();
}

/**
 * @brief Marks the bits at the given indices.
 *
 * Used for coverage and overlap checks: marking an already-set bit reports a
 * duplicate.
 *
 * @param indices Indices of the bits to mark.
 * @param bits The bitset to update.
 * @return `true` if any of the bits was already set.
 */
inline bool UpdateBitSet(const std::vector<std::size_t>& indices, std::vector<bool>& bits) {
  if constexpr (kBelugaEstimationAssertIsArmed) {
    const auto is_valid_index = [&bits](std::size_t index) { return index < bits.size(); };
    const bool indices_in_range = std::all_of(indices.begin(), indices.end(), is_valid_index);
    BELUGA_ESTIMATION_ASSERT(indices_in_range);
  }

  bool overlap = false;
  for (const auto index : indices) {
    if (bits[index]) {
      overlap = true;
    }
    bits[index] = true;
  }
  return overlap;
}

/**
 * @brief Returns the indices of the set bits, in increasing order.
 *
 * @param bits The bitset to enumerate.
 * @return The set-bit indices.
 */
inline std::vector<std::size_t> IndexBitSet(const std::vector<bool>& bits) {
  std::vector<std::size_t> indices;
  indices.reserve(CountBitSet(bits));
  for (std::size_t i = 0; i < bits.size(); ++i) {
    if (bits[i]) {
      indices.push_back(i);
    }
  }
  return indices;
}

}  // namespace beluga_estimation
