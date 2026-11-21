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

#include <chrono>
#include <optional>

#include <gtest/gtest.h>

#include <beluga_estimation/common/time_aware_buffer.hpp>
#include <beluga_estimation/common/types.hpp>

using namespace std::chrono_literals;

namespace {

using beluga_estimation::TimeAwareBuffer;
using beluga_estimation::TimePoint;

// =============================================================================
// Insertion
// =============================================================================

TEST(TimeAwareBufferTest, PushMonotonicAppendsInOrder) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};

  EXPECT_TRUE(buffer.Push(t0, 0));
  EXPECT_TRUE(buffer.Push(t0 + 1s, 1));
  EXPECT_TRUE(buffer.Push(t0 + 2s, 2));

  EXPECT_EQ(buffer.Size(), 3u);
  EXPECT_FALSE(buffer.Empty());

  {
    const auto entry = buffer.Lookup(t0 + 500ms);
    ASSERT_TRUE(entry.has_value());
    EXPECT_EQ(entry->timestamp, t0);
    EXPECT_EQ(entry->value, 0);
  }

  {
    const auto entry = buffer.Lookup(t0 + 1s + 500ms);
    ASSERT_TRUE(entry.has_value());
    EXPECT_EQ(entry->timestamp, t0 + 1s);
    EXPECT_EQ(entry->value, 1);
  }

  {
    const auto entry = buffer.Lookup(t0 + 2s + 500ms);
    ASSERT_TRUE(entry.has_value());
    EXPECT_EQ(entry->timestamp, t0 + 2s);
    EXPECT_EQ(entry->value, 2);
  }
}

TEST(TimeAwareBufferTest, PushDuplicateTimestampReplacesBack) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};

  EXPECT_TRUE(buffer.Push(t0, 1));
  EXPECT_TRUE(buffer.Push(t0, 2));

  EXPECT_EQ(buffer.Size(), 1u);
  const auto entry = buffer.Lookup(t0);
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->value, 2);
}

TEST(TimeAwareBufferTest, PushOutOfOrderIsRejected) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};

  EXPECT_TRUE(buffer.Push(t0, 0));
  EXPECT_TRUE(buffer.Push(t0 + 1s, 1));
  EXPECT_FALSE(buffer.Push(t0, 42));  // Older than the most recent entry.

  EXPECT_EQ(buffer.Size(), 2u);
  const auto entry = buffer.Lookup(t0);
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->value, 0);  // Unchanged.
}

// =============================================================================
// Eviction
// =============================================================================

TEST(TimeAwareBufferTest, EvictRemovesEntriesOlderThanCutoff) {
  TimeAwareBuffer<int> buffer;
  const TimePoint now{};

  EXPECT_TRUE(buffer.Push(now - 10s, 0));
  EXPECT_TRUE(buffer.Push(now - 5s, 1));
  EXPECT_TRUE(buffer.Push(now - 1s, 2));
  buffer.Evict(now, 4s);  // Cutoff at `now - 4s`.

  EXPECT_EQ(buffer.Size(), 1u);
  const auto entry = buffer.Lookup(now - 1s);
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->value, 2);
}

TEST(TimeAwareBufferTest, EvictRetainsEntryExactlyAtCutoff) {
  TimeAwareBuffer<int> buffer;
  const TimePoint now{};
  const auto cutoff = now - 4s;

  EXPECT_TRUE(buffer.Push(now - 10s, 0));
  EXPECT_TRUE(buffer.Push(cutoff, 1));
  buffer.Evict(now, 4s);

  EXPECT_EQ(buffer.Size(), 1u);
  const auto entry = buffer.Lookup(cutoff);
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->value, 1);
}

TEST(TimeAwareBufferTest, EvictOnEmptyBufferIsNoOp) {
  TimeAwareBuffer<int> buffer;
  const TimePoint now{};
  EXPECT_NO_THROW(buffer.Evict(now, 4s));
  EXPECT_TRUE(buffer.Empty());
}

TEST(TimeAwareBufferTest, EvictRemovesAllEntriesWhenCutoffIsNewest) {
  TimeAwareBuffer<int> buffer;
  const TimePoint now{};
  EXPECT_TRUE(buffer.Push(now - 10s, 0));
  EXPECT_TRUE(buffer.Push(now - 5s, 1));

  buffer.Evict(now, 0s);  // Cutoff at `now`: evicts everything older.

  EXPECT_TRUE(buffer.Empty());
  EXPECT_EQ(buffer.Size(), 0u);
}

TEST(TimeAwareBufferTest, PushAfterEvictWorks) {
  TimeAwareBuffer<int> buffer;
  const TimePoint now{};
  EXPECT_TRUE(buffer.Push(now - 10s, 0));
  EXPECT_TRUE(buffer.Push(now - 2s, 1));
  buffer.Evict(now, 4s);  // Removes the entry older than the cutoff.

  EXPECT_EQ(buffer.Size(), 1u);
  EXPECT_TRUE(buffer.Push(now, 2));  // Ordered insert after eviction.
  EXPECT_EQ(buffer.Size(), 2u);

  const auto entry = buffer.Lookup(now);
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->value, 2);
}

// =============================================================================
// Zero-order hold lookup
// =============================================================================

TEST(TimeAwareBufferTest, LookupEmptyBufferReturnsNullopt) {
  TimeAwareBuffer<int> buffer;
  EXPECT_FALSE(buffer.Lookup(TimePoint{}).has_value());
}

TEST(TimeAwareBufferTest, LookupExactMatchReturnsEntry) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};
  EXPECT_TRUE(buffer.Push(t0, 1));
  EXPECT_TRUE(buffer.Push(t0 + 1s, 2));

  const auto entry = buffer.Lookup(t0 + 1s);
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->value, 2);
}

TEST(TimeAwareBufferTest, LookupBetweenEntriesHoldsLatestValue) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};
  EXPECT_TRUE(buffer.Push(t0, 1));
  EXPECT_TRUE(buffer.Push(t0 + 2s, 2));

  const auto entry = buffer.Lookup(t0 + 1s);
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->value, 1);  // Zero-order hold.
}

TEST(TimeAwareBufferTest, LookupTooEarlyReturnsNullopt) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};
  EXPECT_TRUE(buffer.Push(t0, 1));

  EXPECT_FALSE(buffer.Lookup(t0 - 1s).has_value());
}

// =============================================================================
// Functional lookup
// =============================================================================

TEST(TimeAwareBufferTest, LookupFunctionalInterpolatesBetweenEntries) {
  TimeAwareBuffer<double> buffer;
  const TimePoint t0{};
  EXPECT_TRUE(buffer.Push(t0, 0.0));
  EXPECT_TRUE(buffer.Push(t0 + 2s, 2.0));

  auto first_order_hold = [](const TimeAwareBuffer<double>::Entry* before, const TimeAwareBuffer<double>::Entry* after,
                             TimePoint timestamp) -> std::optional<double> {
    if (before == nullptr || after == nullptr) {
      return std::nullopt;  // Lookup too early or too late.
    }
    const double dt = std::chrono::duration<double>(after->timestamp - before->timestamp).count();
    const double alpha = std::chrono::duration<double>(timestamp - before->timestamp).count() / dt;
    return (1.0 - alpha) * before->value + alpha * after->value;
  };
  const auto result = buffer.Lookup(t0 + 1s, first_order_hold);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(*result, 1.0, 1e-9);
}

TEST(TimeAwareBufferTest, LookupFunctionalExactMatchPassesSameEntry) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};
  EXPECT_TRUE(buffer.Push(t0, 1));
  EXPECT_TRUE(buffer.Push(t0 + 1s, 2));

  // An exact match passes the same entry on both sides of the callback,
  // whether the match is the first or the last entry.
  const auto same_entry_only = [](const TimeAwareBuffer<int>::Entry* before, const TimeAwareBuffer<int>::Entry* after,
                                  TimePoint) -> std::optional<int> {
    if (before == nullptr || after == nullptr) {
      return std::nullopt;
    }
    EXPECT_EQ(before, after);  // Same entry on both sides.
    return before->value;
  };
  for (const auto& [time, expected] : {std::pair{t0, 1}, std::pair{t0 + 1s, 2}}) {
    const auto result = buffer.Lookup(time, same_entry_only);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, expected);
  }
}

TEST(TimeAwareBufferTest, LookupFunctionalMayExtrapolateWhenTooEarly) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};
  EXPECT_TRUE(buffer.Push(t0, 7));

  // The default zero-order hold rejects lookups before the first entry...
  EXPECT_FALSE(buffer.Lookup(t0 - 1s).has_value());

  // ...but a user-defined function may choose to extrapolate instead.
  auto zero_order_hold_with_extrapolation = [](const TimeAwareBuffer<int>::Entry* before,
                                               const TimeAwareBuffer<int>::Entry* after,
                                               TimePoint) -> std::optional<int> {
    if (before != nullptr) {
      return before->value;  // Hold the last known value.
    }
    if (after != nullptr) {
      return after->value;  // Hold the first known value.
    }
    return std::nullopt;  // No entries at all.
  };
  const auto result = buffer.Lookup(t0 - 1s, zero_order_hold_with_extrapolation);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, 7);
}

TEST(TimeAwareBufferTest, LookupFunctionalOnEmptyMayStillReturn) {
  TimeAwareBuffer<int> buffer;
  auto zero_order_hold_with_fallback = [](const TimeAwareBuffer<int>::Entry* before,
                                          const TimeAwareBuffer<int>::Entry* after, TimePoint) -> int {
    if (before == nullptr) {
      return 0;  // No entries at all.
    }
    return before->value;  // Hold the last known value.
  };
  const int result = buffer.Lookup(TimePoint{}, zero_order_hold_with_fallback);
  EXPECT_EQ(result, 0);
}

// =============================================================================
// Clear
// =============================================================================

TEST(TimeAwareBufferTest, ClearRemovesAllEntries) {
  TimeAwareBuffer<int> buffer;
  const TimePoint t0{};
  EXPECT_TRUE(buffer.Push(t0, 1));
  EXPECT_TRUE(buffer.Push(t0 + 1s, 2));

  buffer.Clear();

  EXPECT_TRUE(buffer.Empty());
  EXPECT_EQ(buffer.Size(), 0u);
  EXPECT_FALSE(buffer.Lookup(t0).has_value());
}

}  // namespace
