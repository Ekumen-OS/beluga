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

#include <cstddef>
#include <optional>

#include <beluga_estimation/common/views.hpp>

namespace beluga_estimation {

/// @brief Represents a contiguous slice of a sequence of indices.
///
/// A slice covers the half-open range `[start, end)` of indices.
/// Both `start` and `end` are optional: a missing `start` defaults to the
/// beginning of the sequence (index 0) and a missing `end` defaults to the
/// end of the sequence.
struct contiguous_slice {
  /// @brief Factory method to create a slice that starts at a given index.
  /// @param start The start index of the slice.
  /// @return A slice covering `[start, ...)`.
  static contiguous_slice From(std::size_t start) { return contiguous_slice{start, std::nullopt}; }

  /// @brief Factory method to create a slice that ends at a given index.
  /// @param end The end index of the slice.
  /// @return A slice covering `[0, end)`.
  static contiguous_slice Until(std::size_t end) { return contiguous_slice{std::nullopt, end}; }

  /// @brief Factory method to create a slice from a start index and a count.
  /// @param start The start index of the slice.
  /// @param count The number of indices covered by the slice.
  /// @return A slice covering `[start, start + count)`.
  static contiguous_slice Span(std::size_t start, std::size_t count) { return contiguous_slice{start, start + count}; }

  /// @brief The optional start index of the slice.
  std::optional<std::size_t> start;
  /// @brief The optional end index of the slice (exclusive).
  std::optional<std::size_t> end;
};

/// @brief Checks if @p slices are disjoint.
///
/// @pre Slices must be sorted by start.
/// @param slices A view over a collection of slices.
/// @return true if all pairs of slices are disjoint,
/// false if any pair may be overlapping. Note that
/// unsorted slices can yield false negatives.
inline bool disjoint(beluga_estimation::view<const contiguous_slice> slices) {
  if (slices.size() > 1) {
    for (std::size_t i = 0; i < slices.size() - 1; ++i) {
      const auto& prev = slices[i];
      const auto& next = slices[i + 1];
      if (!prev.end || !next.start || *prev.end > *next.start) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace beluga_estimation
