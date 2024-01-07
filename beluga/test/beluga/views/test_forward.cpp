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

#include "beluga/views/forward.hpp"

#include <range/v3/algorithm/equal.hpp>
#include <range/v3/algorithm/find.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/transform.hpp>

namespace {

TEST(ForwardView, ConceptChecksFromContiguousRange) {
  auto input = std::array{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = beluga::views::forward(input);

  static_assert(ranges::common_range<decltype(input)>);
  static_assert(!ranges::common_range<decltype(output)>);

  static_assert(!ranges::viewable_range<decltype(input)>);
  static_assert(ranges::viewable_range<decltype(output)>);

  static_assert(ranges::forward_range<decltype(input)>);
  static_assert(ranges::forward_range<decltype(output)>);

  static_assert(ranges::sized_range<decltype(input)>);
  static_assert(ranges::sized_range<decltype(output)>);

  static_assert(ranges::bidirectional_range<decltype(input)>);
  static_assert(!ranges::bidirectional_range<decltype(output)>);

  static_assert(ranges::random_access_range<decltype(input)>);
  static_assert(!ranges::random_access_range<decltype(output)>);

  static_assert(ranges::contiguous_range<decltype(input)>);
  static_assert(!ranges::contiguous_range<decltype(output)>);
}

TEST(ForwardView, ConceptChecksFromInfiniteRange) {
  auto input = ranges::views::generate([]() { return 1; });
  auto output = input | beluga::views::forward;

  static_assert(!ranges::common_range<decltype(input)>);
  static_assert(!ranges::common_range<decltype(output)>);

  static_assert(ranges::viewable_range<decltype(input)>);
  static_assert(ranges::viewable_range<decltype(output)>);

  static_assert(!ranges::forward_range<decltype(input)>);
  static_assert(!ranges::forward_range<decltype(output)>);

  static_assert(!ranges::sized_range<decltype(input)>);
  static_assert(!ranges::sized_range<decltype(output)>);

  static_assert(!ranges::bidirectional_range<decltype(input)>);
  static_assert(!ranges::bidirectional_range<decltype(output)>);

  static_assert(!ranges::random_access_range<decltype(input)>);
  static_assert(!ranges::random_access_range<decltype(output)>);

  static_assert(!ranges::contiguous_range<decltype(input)>);
  static_assert(!ranges::contiguous_range<decltype(output)>);
}

TEST(ForwardView, AllElements) {
  auto input = std::array{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  auto output = beluga::views::forward(input);
  ranges::equal(input, output);
}

TEST(ForwardView, BorrowedRange) {
  auto input = std::array{42};
  const auto create_view = [&]() { return input | beluga::views::forward; };
  auto it = ranges::find(create_view(), 42);
  ASSERT_EQ(*it, 42);  // the iterator is not dangling after the view is destroyed
}

inline constexpr auto identity = [](auto&& t) noexcept { return std::forward<decltype(t)>(t); };

TEST(ForwardView, NonBorrowedRange) {
  auto input = std::array{42};
  const auto create_view = [&]() { return input | ranges::views::transform(identity) | beluga::views::forward; };
  auto it = ranges::find(create_view(), 42);
  static_assert(std::is_same_v<decltype(it), ranges::dangling>);  // the iterator is dangling since transform is not a
                                                                  // borrowed range
}

}  // namespace
