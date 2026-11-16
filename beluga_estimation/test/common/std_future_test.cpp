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

#include <array>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/views.hpp>

namespace {

using beluga_estimation::std_future::stop_source;
using beluga_estimation::std_future::stop_token;

TEST(StopToken, DefaultConstructedHasNoState) {
  const stop_token token;
  EXPECT_FALSE(token.stop_possible());
  EXPECT_FALSE(token.stop_requested());
}

TEST(StopToken, EqualityOfDefaultConstructedTokens) {
  const stop_token a;
  const stop_token b;
  EXPECT_EQ(a, b);
}

TEST(StopSource, DefaultConstructedHasState) {
  const stop_source source;
  EXPECT_TRUE(source.stop_possible());
  EXPECT_FALSE(source.stop_requested());
}

TEST(StopSource, NoStopStateConstructedHasNoState) {
  const stop_source source{stop_source::nostopstate};
  EXPECT_FALSE(source.stop_possible());
  EXPECT_FALSE(source.stop_requested());
}

TEST(StopSource, RequestStopReturnsTrueFirstTime) {
  stop_source source;
  EXPECT_TRUE(source.request_stop());
}

TEST(StopSource, RequestStopReturnsFalseOnSubsequentCalls) {
  stop_source source;
  source.request_stop();
  EXPECT_FALSE(source.request_stop());
}

TEST(StopSource, RequestStopReturnsFalseWithNoState) {
  stop_source source{stop_source::nostopstate};
  EXPECT_FALSE(source.request_stop());
}

TEST(StopSource, StopRequestedAfterRequestStop) {
  stop_source source;
  source.request_stop();
  EXPECT_TRUE(source.stop_requested());
}

TEST(StopSource, EqualityOfCopiedSources) {
  const stop_source a;
  const stop_source b = a;
  EXPECT_EQ(a, b);
}

TEST(StopSource, InequalityOfIndependentSources) {
  const stop_source a;
  const stop_source b;
  EXPECT_NE(a, b);
}

TEST(StopSourceToken, TokenIsPossibleWhenSourceHasState) {
  const stop_source source;
  const stop_token token = source.get_token();
  EXPECT_TRUE(token.stop_possible());
  EXPECT_FALSE(token.stop_requested());
}

TEST(StopSourceToken, TokenIsNotPossibleWhenSourceHasNoState) {
  const stop_source source{stop_source::nostopstate};
  const stop_token token = source.get_token();
  EXPECT_FALSE(token.stop_possible());
}

TEST(StopSourceToken, TokenReflectsStopRequest) {
  stop_source source;
  const stop_token token = source.get_token();
  EXPECT_FALSE(token.stop_requested());
  source.request_stop();
  EXPECT_TRUE(token.stop_requested());
}

TEST(StopSourceToken, MultipleTokensShareState) {
  stop_source source;
  const stop_token token_a = source.get_token();
  const stop_token token_b = source.get_token();
  EXPECT_EQ(token_a, token_b);
  source.request_stop();
  EXPECT_TRUE(token_a.stop_requested());
  EXPECT_TRUE(token_b.stop_requested());
}

TEST(StopSourceToken, TokenRemainsValidAfterSourceIsDestroyed) {
  stop_token token;
  {
    stop_source source;
    token = source.get_token();
    source.request_stop();
  }
  EXPECT_TRUE(token.stop_possible());
  EXPECT_TRUE(token.stop_requested());
}

// --- std_future::copy tests ---

TEST(MdspanCopy, FullyContiguousRowMajorCopiesCorrectly) {
  // Both views are fully contiguous row-major (stride(0) == num_columns,
  // stride(1) == 1): a single memmove copies the whole matrix.
  std::vector<double> src_data = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  auto src = beluga_estimation::std_future::mdspan<const double, 2>(src_data.data(), 2, 3);
  std::vector<double> dst_data(6, 0.0);
  auto dst = beluga_estimation::std_future::mdspan<double, 2>(dst_data.data(), 2, 3);

  beluga_estimation::std_future::copy(dst, src);

  EXPECT_DOUBLE_EQ(dst(1, 2), 6.0);
  EXPECT_DOUBLE_EQ(dst(0, 1), 2.0);
}

TEST(MdspanCopy, StridedRowsCopiesPerRow) {
  // Destination is contiguous 3x3; source rows are stored with a stride of 5.
  std::vector<double> src_data(15, 0.0);
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      src_data[row * 5 + column] = static_cast<double>(row * 3 + column + 1);
    }
  }
  using mapping_type = beluga_estimation::std_future::mdspan<double, 2>::mapping_type;
  mapping_type src_mapping(mapping_type::extents_type(3, 3), std::array<std::size_t, 2>{5, 1});
  auto src = beluga_estimation::std_future::mdspan<const double, 2>(src_data.data(), src_mapping);
  std::vector<double> dst_data(9, 0.0);
  auto dst = beluga_estimation::std_future::mdspan<double, 2>(dst_data.data(), 3, 3);

  beluga_estimation::std_future::copy(dst, src);

  EXPECT_DOUBLE_EQ(dst(2, 2), 9.0);
  EXPECT_DOUBLE_EQ(dst(1, 0), 4.0);
}

TEST(MdspanCopy, FullyContiguousColumnMajorCopiesCorrectly) {
  // Both views are fully contiguous column-major (stride(0) == 1,
  // stride(1) == num_rows): a single memmove copies the whole matrix.
  using mapping_type = beluga_estimation::std_future::mdspan<double, 2>::mapping_type;
  mapping_type src_mapping(mapping_type::extents_type(2, 3), std::array<std::size_t, 2>{1, 2});
  std::vector<double> src_data(6, 0.0);
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      src_data[row + column * 2] = static_cast<double>(row * 3 + column + 1);
    }
  }
  auto src = beluga_estimation::std_future::mdspan<const double, 2>(src_data.data(), src_mapping);
  mapping_type dst_mapping(mapping_type::extents_type(2, 3), std::array<std::size_t, 2>{1, 2});
  std::vector<double> dst_data(6, 0.0);
  auto dst = beluga_estimation::std_future::mdspan<double, 2>(dst_data.data(), dst_mapping);

  beluga_estimation::std_future::copy(dst, src);

  EXPECT_DOUBLE_EQ(dst(1, 2), 6.0);
  EXPECT_DOUBLE_EQ(dst(0, 1), 2.0);
}

TEST(MdspanCopy, StridedColumnsCopiesPerColumn) {
  // Both views are column-contiguous (stride(0) == 1) but not fully
  // contiguous (stride(1) != num_rows): one memmove per column.
  using mapping_type = beluga_estimation::std_future::mdspan<double, 2>::mapping_type;
  mapping_type src_mapping(mapping_type::extents_type(3, 2), std::array<std::size_t, 2>{1, 4});
  std::vector<double> src_data(8, 0.0);
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 2; ++column) {
      src_data[row + column * 4] = static_cast<double>(row * 2 + column + 1);
    }
  }
  auto src = beluga_estimation::std_future::mdspan<const double, 2>(src_data.data(), src_mapping);
  mapping_type dst_mapping(mapping_type::extents_type(3, 2), std::array<std::size_t, 2>{1, 4});
  std::vector<double> dst_data(8, 0.0);
  auto dst = beluga_estimation::std_future::mdspan<double, 2>(dst_data.data(), dst_mapping);

  beluga_estimation::std_future::copy(dst, src);

  EXPECT_DOUBLE_EQ(dst(2, 1), 6.0);
  EXPECT_DOUBLE_EQ(dst(0, 0), 1.0);
}

TEST(MdspanCopy, FullyContiguousOverlappingViewsCopyCorrectly) {
  // Overlapping fully contiguous views are safe: a single memmove behaves
  // like std::memmove. This exercises a left shift by one element.
  std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0};
  auto src = beluga_estimation::std_future::mdspan<const double, 2>(data.data(), 2, 3);
  auto dst = beluga_estimation::std_future::mdspan<double, 2>(data.data() + 1, 2, 3);

  beluga_estimation::std_future::copy(dst, src);

  // dst must contain the original src values (left shift by one element).
  EXPECT_DOUBLE_EQ(dst(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(dst(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(dst(0, 2), 3.0);
  EXPECT_DOUBLE_EQ(dst(1, 0), 4.0);
  EXPECT_DOUBLE_EQ(dst(1, 1), 5.0);
  EXPECT_DOUBLE_EQ(dst(1, 2), 6.0);
}

TEST(MdspanCopy, NonContiguousFallsBackToElementWise) {
  // Neither rows nor columns are contiguous (strides 2, 2).
  using mapping_type = beluga_estimation::std_future::mdspan<double, 2>::mapping_type;
  mapping_type src_mapping(mapping_type::extents_type(2, 2), std::array<std::size_t, 2>{2, 2});
  std::vector<double> src_data = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  auto src = beluga_estimation::std_future::mdspan<const double, 2>(src_data.data(), src_mapping);
  std::vector<double> dst_data(4, 0.0);
  auto dst = beluga_estimation::std_future::mdspan<double, 2>(dst_data.data(), 2, 2);

  beluga_estimation::std_future::copy(dst, src);

  // With strides (2, 2), src(1, 1) reads src_data[4].
  EXPECT_DOUBLE_EQ(dst(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(dst(1, 1), 5.0);
}

TEST(MdspanCopy, EmptyExtentsIsNoOp) {
  // An empty copy must leave the underlying buffer untouched. The empty
  // views overlay the buffer they are asserted against, so a broken empty
  // case (e.g. an out-of-bounds write) would be caught.
  std::vector<double> buffer(6, 1.0);
  auto empty_src = beluga_estimation::std_future::mdspan<const double, 2>(buffer.data(), 0, 3);
  auto empty_dst = beluga_estimation::std_future::mdspan<double, 2>(buffer.data(), 0, 3);

  beluga_estimation::std_future::copy(empty_dst, empty_src);

  EXPECT_EQ(buffer, (std::vector<double>(6, 1.0)));
}

// --- mdspan converting constructor tests ---

TEST(Mdspan, MutableConvertsToConst) {
  std::vector<double> data = {1.0, 2.0, 3.0, 4.0};
  beluga_estimation::std_future::mdspan<double, 2> mutable_view(data.data(), 2, 2);
  beluga_estimation::std_future::mdspan<const double, 2> const_view = mutable_view;

  EXPECT_DOUBLE_EQ(const_view(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(const_view(1, 1), 4.0);
}

}  // namespace
