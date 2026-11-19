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

#include <vector>

#include <gtest/gtest.h>

#include <beluga_estimation/common/bitset.hpp>

namespace {

TEST(BitSet, ClearBitSetResetsEveryBit) {
  std::vector<bool> bits = {true, false, true};
  beluga_estimation::ClearBitSet(bits);
  EXPECT_EQ(bits, std::vector<bool>({false, false, false}));
}

TEST(BitSet, AnyBitSet) {
  EXPECT_FALSE(beluga_estimation::AnyBitSet({}));
  EXPECT_FALSE(beluga_estimation::AnyBitSet({false, false}));
  EXPECT_TRUE(beluga_estimation::AnyBitSet({false, true}));
}

TEST(BitSet, CountBitSet) {
  EXPECT_EQ(beluga_estimation::CountBitSet({}), 0u);
  EXPECT_EQ(beluga_estimation::CountBitSet({false, false}), 0u);
  EXPECT_EQ(beluga_estimation::CountBitSet({true, false, true}), 2u);
}

TEST(BitSet, AllBitSet) {
  EXPECT_TRUE(beluga_estimation::AllBitSet({}));
  EXPECT_TRUE(beluga_estimation::AllBitSet({true, true}));
  EXPECT_FALSE(beluga_estimation::AllBitSet({true, false}));
}

TEST(BitSet, UpdateBitSetReportsOverlap) {
  std::vector<bool> bits(4, false);
  EXPECT_FALSE(beluga_estimation::UpdateBitSet({1, 3}, bits));
  EXPECT_EQ(bits, std::vector<bool>({false, true, false, true}));
  // Marking a set bit again reports the overlap.
  EXPECT_TRUE(beluga_estimation::UpdateBitSet({0, 3}, bits));
  EXPECT_EQ(bits, std::vector<bool>({true, true, false, true}));
}

TEST(BitSet, IndexBitSetReturnsSetIndicesInOrder) {
  EXPECT_EQ(beluga_estimation::IndexBitSet({}), std::vector<std::size_t>({}));
  EXPECT_EQ(beluga_estimation::IndexBitSet({false, false, false}), std::vector<std::size_t>({}));
  EXPECT_EQ(beluga_estimation::IndexBitSet({true, false, true, true}), std::vector<std::size_t>({0, 2, 3}));
}

}  // namespace
