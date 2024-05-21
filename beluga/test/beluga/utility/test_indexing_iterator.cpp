// Copyright 2024 Ekumen, Inc.
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iterator>
#include <type_traits>
#include <vector>

#include <range/v3/iterator/concepts.hpp>

#include "beluga/utility/indexing_iterator.hpp"

namespace {

TEST(IndexingIterator, IteratorTraits) {
  using Iterator = beluga::IndexingIterator<std::vector<int>>;
  static_assert(ranges::random_access_iterator<Iterator>);
  static_assert(std::is_signed_v<typename Iterator::difference_type>);
  static_assert(std::is_same_v<typename Iterator::value_type, int>);
  static_assert(std::is_same_v<typename Iterator::reference, int&>);
  static_assert(std::is_same_v<typename Iterator::pointer, int*>);
  using ssize_type = std::make_signed_t<typename std::vector<int>::size_type>;
  static_assert(std::is_same_v<typename Iterator::difference_type, ssize_type>);
}

TEST(IndexingIterator, ConstIteratorTraits) {
  using Iterator = beluga::IndexingIterator<const std::vector<int>>;
  static_assert(ranges::random_access_iterator<Iterator>);
  static_assert(std::is_same_v<typename Iterator::value_type, int>);
  static_assert(std::is_same_v<typename Iterator::reference, const int&>);
  static_assert(std::is_same_v<typename Iterator::pointer, const int*>);
  using ssize_type = std::make_signed_t<typename std::vector<int>::size_type>;
  static_assert(std::is_same_v<typename Iterator::difference_type, ssize_type>);
}

TEST(IndexingIterator, Arithmetic) {
  const auto sequence = std::vector{0, 1};
  auto begin = beluga::IndexingIterator(sequence);
  auto end = beluga::IndexingIterator(sequence, sequence.size());
  EXPECT_EQ(std::next(begin, std::distance(begin, end)), end);
  EXPECT_EQ(std::prev(end, std::distance(begin, end)), begin);
  EXPECT_EQ(std::abs(end - begin), std::abs(begin - end));
}

TEST(IndexingIterator, EqualityPreserving) {
  const auto sequence = std::vector{0, 1};
  auto begin = beluga::IndexingIterator(sequence);
  auto end = beluga::IndexingIterator(sequence, sequence.size());
  EXPECT_EQ(begin, --(begin + 1));
  EXPECT_EQ(end, ++(end - 1));
}

TEST(IndexingIterator, Iterate) {
  const auto expected_sequence = std::vector{3, 0, -1, 5, 8, -2};
  const auto sequence = std::vector(
      beluga::IndexingIterator(expected_sequence),
      beluga::IndexingIterator(expected_sequence, expected_sequence.size()));
  ASSERT_THAT(sequence, ::testing::ContainerEq(expected_sequence));
}

TEST(IndexingIterator, IterateAndMutate) {
  const auto input_sequence = std::vector{1, 2, 3, 4, 5, 6};
  auto output_sequence = std::vector<int>(input_sequence.size());
  std::transform(
      input_sequence.begin(), input_sequence.end(), input_sequence.rbegin(), beluga::IndexingIterator(output_sequence),
      std::plus<>{});
  ASSERT_THAT(output_sequence, ::testing::Each(::testing::Eq(7)));
}

TEST(IndexingIterator, IterateAndMove) {
  const auto expected_sequence = std::vector{std::vector{1, 3, 2, 4, 3, 5}, std::vector{1, -1, 0, 1, -1}};
  auto input_sequence = expected_sequence;
  const auto output_sequence = std::vector(
      std::make_move_iterator(beluga::IndexingIterator(input_sequence)),
      std::make_move_iterator(beluga::IndexingIterator(input_sequence, input_sequence.size())));
  EXPECT_THAT(input_sequence, ::testing::Each(::testing::IsEmpty()));
  ASSERT_THAT(output_sequence, ::testing::ContainerEq(expected_sequence));
}

}  // namespace
