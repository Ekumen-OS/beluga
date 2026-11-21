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

#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include <beluga_estimation/common/slice.hpp>

namespace {

using beluga_estimation::contiguous_slice;

// --- contiguous_slice factories ---

TEST(ContiguousSlice, SpanCreatesBoundedSlice) {
  auto slice = contiguous_slice::Span(5, 3);
  ASSERT_TRUE(slice.start);
  ASSERT_TRUE(slice.end);
  EXPECT_EQ(*slice.start, 5);
  EXPECT_EQ(*slice.end, 8);
}

TEST(ContiguousSlice, FromCreatesOpenEndedSlice) {
  auto slice = contiguous_slice::From(5);
  ASSERT_TRUE(slice.start);
  ASSERT_FALSE(slice.end.has_value());
  EXPECT_EQ(*slice.start, 5);
}

TEST(ContiguousSlice, UntilCreatesOpenEndedSlice) {
  auto slice = contiguous_slice::Until(5);
  ASSERT_FALSE(slice.start.has_value());
  ASSERT_TRUE(slice.end);
  EXPECT_EQ(*slice.end, 5);
}

TEST(ContiguousSlice, AggregateConstruction) {
  auto slice = contiguous_slice{5, 8};
  ASSERT_TRUE(slice.start);
  ASSERT_TRUE(slice.end);
  EXPECT_EQ(*slice.start, 5);
  EXPECT_EQ(*slice.end, 8);
}

// --- disjoint: bounded slices ---

TEST(Disjoint, BoundedNonOverlapping) {
  // [0, 3) and [3, 6) — adjacent, disjoint.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{0, 3}, contiguous_slice{3, 6}};
    EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  // [0, 3) and [5, 8) — gap, disjoint.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{0, 3}, contiguous_slice{5, 8}};
    EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
}

TEST(Disjoint, BoundedOverlapping) {
  // [0, 5) and [3, 8) — overlap at [3, 5).
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{0, 5}, contiguous_slice{3, 8}};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  // [0, 5) and [0, 5) — identical.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{0, 5}, contiguous_slice{0, 5}};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  // [0, 10) and [2, 4) — one contains the other.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{0, 10}, contiguous_slice{2, 4}};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
}

// --- disjoint: open-ended slices ---

TEST(Disjoint, OpenEndedLeft) {
  // From(5) = [5, ∞). [3, 6) and [5, ∞) — overlap at [5, 6).
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{3, 6}, contiguous_slice::From(5)};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  // From(5) = [5, ∞). [1, 5) and [5, ∞) — disjoint (5 not in [1, 5)).
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{1, 5}, contiguous_slice::From(5)};
    EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
}

TEST(Disjoint, OpenEndedRight) {
  // Until(5) = [0, 5). [0, 5) and [3, 6) — overlap at [3, 5).
  {
    std::vector<contiguous_slice> slices = {contiguous_slice::Until(5), contiguous_slice{3, 6}};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  // Until(5) = [0, 5). [0, 5) and [5, 8) — disjoint.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice::Until(5), contiguous_slice{5, 8}};
    EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
}

TEST(Disjoint, BothOpenEnded) {
  // Until(3) = [0, 3). From(5) = [5, ∞). Disjoint (gap [3, 5)).
  {
    std::vector<contiguous_slice> slices = {contiguous_slice::Until(3), contiguous_slice::From(5)};
    EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  // Until(5) = [0, 5). From(3) = [3, ∞). Overlap at [3, 5).
  {
    std::vector<contiguous_slice> slices = {contiguous_slice::Until(5), contiguous_slice::From(3)};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
}

TEST(Disjoint, FullyUnbounded) {
  // {nullopt, nullopt} = [0, ∞) and anything — always overlap.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{std::nullopt, std::nullopt}, contiguous_slice{0, 5}};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  {
    std::vector<contiguous_slice> slices = {
        contiguous_slice{std::nullopt, std::nullopt}, contiguous_slice{std::nullopt, std::nullopt}};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
}

TEST(Disjoint, SameSideUnbounded) {
  // From(3) = [3, ∞). From(5) = [5, ∞). Both go to +∞ — not disjoint.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice::From(3), contiguous_slice::From(5)};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  // Until(3) = [0, 3). Until(5) = [0, 5). Both start at the beginning — not disjoint.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice::Until(3), contiguous_slice::Until(5)};
    EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
}

// --- disjoint: view overload ---

TEST(DisjointView, EmptyCollection) {
  std::vector<contiguous_slice> empty;
  EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(empty)));
}

TEST(DisjointView, SingleSlice) {
  std::vector<contiguous_slice> slices = {contiguous_slice{0, 5}};
  EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
}

TEST(DisjointView, EmptySliceOverlapsNothing) {
  // An empty slice {0, 0} covers no indices, so it is disjoint from anything.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{0, 0}, contiguous_slice{0, 5}};
    EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
  // An empty slice at the end of a sequence of slices.
  {
    std::vector<contiguous_slice> slices = {contiguous_slice{0, 3}, contiguous_slice{3, 3}};
    EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
  }
}

TEST(DisjointView, PairwiseDisjoint) {
  std::vector<contiguous_slice> slices = {contiguous_slice{0, 3}, contiguous_slice{3, 6}, contiguous_slice{6, 9}};
  EXPECT_TRUE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
}

TEST(DisjointView, PairwiseOverlap) {
  std::vector<contiguous_slice> slices = {contiguous_slice{0, 4}, contiguous_slice{2, 6}, contiguous_slice{6, 9}};
  EXPECT_FALSE(beluga_estimation::disjoint(beluga_estimation::to_view(slices)));
}

}  // namespace
