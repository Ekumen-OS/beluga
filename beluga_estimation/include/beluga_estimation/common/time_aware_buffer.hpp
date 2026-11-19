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
 * @brief Defines a generic time-aware buffer with ordered insertion, time-based
 * eviction and time-keyed lookup.
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <deque>
#include <iterator>
#include <optional>
#include <type_traits>
#include <utility>

#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation {

/**
 * @brief A buffer of timestamped values with eviction and time-keyed lookup.
 *
 * Entries are kept in chronological order. Insertion is expected to be ordered
 * (non-decreasing timestamps): a value pushed at the same timestamp as the most
 * recent entry replaces it, and a value pushed with an older timestamp is
 * rejected as an error condition and ignored. This assumes a single message
 * source; supporting multiple out-of-order sources is out of scope for now.
 *
 * Lookups are performed with `std::lower_bound`, taking advantage of the
 * random access storage. This class is not thread-safe; callers must
 * synchronize access externally when used across threads.
 *
 * @tparam T Value type. Must be copyable for `Lookup(TimePoint)` (zero-order
 * hold) to return an entry by value, and move constructible/assignable for
 * `Push`.
 */
template <typename T>
class TimeAwareBuffer {
 public:
  /// @brief A timestamped entry in the buffer.
  struct Entry {
    TimePoint timestamp;  ///< Timestamp associated with the value.
    T value;              ///< Stored value.
  };

  /// @brief Default constructor.
  TimeAwareBuffer() = default;

  /// @brief Returns true if the buffer contains no entries.
  [[nodiscard]] bool Empty() const { return buffer_.empty(); }

  /// @brief Returns the number of entries in the buffer.
  [[nodiscard]] std::size_t Size() const { return buffer_.size(); }

  /// @brief Removes all entries from the buffer.
  void Clear() { buffer_.clear(); }

  /**
   * @brief Inserts a value at the given timestamp.
   *
   * If the timestamp is newer than the most recent entry, the value is
   * appended. If it matches the most recent entry timestamp, the stored value
   * is replaced. If it is older than the most recent entry timestamp, the push
   * is rejected as an out-of-order error condition.
   *
   * @param timestamp The timestamp associated with the value.
   * @param value The value to store.
   * @return true if the value was inserted or replaced, false if it was
   * rejected as out of order.
   */
  [[nodiscard]] bool Push(TimePoint timestamp, T value) {
    if (buffer_.empty()) {
      buffer_.push_back(Entry{timestamp, std::move(value)});
      return true;
    }
    const auto& back = buffer_.back();
    if (timestamp > back.timestamp) {
      buffer_.push_back(Entry{timestamp, std::move(value)});
      return true;
    }
    if (timestamp == back.timestamp) {
      buffer_.back().value = std::move(value);
      return true;
    }
    return false;
  }

  /**
   * @brief Evicts entries older than `now - max_age`.
   *
   * Entries whose timestamp is strictly older than the eviction cutoff are
   * removed. An entry exactly at the cutoff is retained.
   *
   * @param now The current time, used to compute the eviction cutoff.
   * @param max_age Maximum age of retained entries. Must be non-negative.
   */
  void Evict(TimePoint now, std::chrono::nanoseconds max_age) {
    const TimePoint cutoff = now - max_age;
    auto first_kept = LowerBound(buffer_, cutoff);
    buffer_.erase(buffer_.begin(), first_kept);
  }

  /**
   * @brief Looks up the entry at or immediately before the given timestamp.
   *
   * Implements zero-order hold semantics: an exact timestamp match returns the
   * matching entry, and a timestamp between two entries returns the most recent
   * entry at or before it. Lookups that are too early (before the first entry)
   * return `std::nullopt`.
   *
   * @param timestamp The timestamp to look up.
   * @return The held entry, or `std::nullopt` if the buffer is empty or the
   * timestamp falls before all buffered range.
   */
  [[nodiscard]] std::optional<Entry> Lookup(TimePoint timestamp) const {
    return Lookup(timestamp, [](const Entry* before, const Entry* after, TimePoint) -> std::optional<Entry> {
      if (before == nullptr) {
        return std::nullopt;  // Lookup too early.
      }
      return *before;  // Zero-order hold: most recent entry at or before the timestamp.
    });
  }

  /**
   * @brief Looks up the pair of entries bracketing the given timestamp.
   *
   * Invokes `func` with pointers to the most recent entry at or before the
   * timestamp and the earliest entry at or after it. Either pointer may be
   * `nullptr` when the timestamp falls outside the buffered range, including
   * the empty range case, so the provided function may choose to extrapolate
   * or reject the lookup. An exact timestamp match passes the same entry for
   * both sides. The pointers are valid only for the duration of the call; any
   * mutation of the buffer invalidates them.
   *
   * @tparam Func Function type accepting `(const Entry*, const Entry*, TimePoint)`.
   * @param timestamp The timestamp to look up.
   * @param func The function to invoke with the bracketing pair.
   * @return The result of `func`.
   */
  template <typename Func>
  [[nodiscard]] auto Lookup(TimePoint timestamp, Func&& func) const {
    auto it = LowerBound(buffer_, timestamp);
    if (it != buffer_.end() && it->timestamp == timestamp) {
      const Entry* exact = std::addressof(*it);
      return func(exact, exact, timestamp);
    }
    const Entry* before = it != buffer_.begin() ? std::addressof(*std::prev(it)) : nullptr;
    const Entry* after = it != buffer_.end() ? std::addressof(*it) : nullptr;
    return func(before, after, timestamp);
  }

 private:
  static auto LowerBound(const std::deque<Entry>& buffer, TimePoint timestamp) {
    return std::lower_bound(buffer.begin(), buffer.end(), timestamp, [](const Entry& entry, const TimePoint& t) {
      return entry.timestamp < t;
    });
  }

  std::deque<Entry> buffer_;
};

}  // namespace beluga_estimation
