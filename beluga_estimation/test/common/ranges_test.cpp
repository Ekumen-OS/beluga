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

#include <cstddef>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <range/v3/iterator/operations.hpp>
#include <range/v3/range/access.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/ranges.hpp>
#include <beluga_estimation/common/views.hpp>

namespace beluga_estimation {
namespace {

// Materializes the rows of a transposed view and compares them with the
// expected rows, one vector per row.
template <typename View>
void ExpectTranspose(View& view, std::vector<std::vector<int>> expected_rows) {
  std::vector<std::vector<int>> rows;
  for (auto row : view) {
    rows.push_back(row | ranges::to_vector);
  }
  ASSERT_EQ(rows.size(), expected_rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    EXPECT_EQ(rows[i], expected_rows[i]);
  }
}

TEST(TransposeView, TransposesColumnsIntoRows) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};

  auto view = views::transpose(columns);
  ExpectTranspose(view, {{1, 10}, {2, 20}, {3, 30}});
}

TEST(TransposeView, TruncatesToShortestColumn) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20}};  // second is shorter

  auto view = views::transpose(columns);
  ExpectTranspose(view, {{1, 10}, {2, 20}});
}

TEST(TransposeView, SinglePassColumns) {
  // Type-erased views: their iterators alias the view's storage, so the
  // views (and their sources) must outlive the transpose. transpose_view owns
  // its columns by value, and the views are moved into the vector below.
  std::vector<std::vector<int>> sources{{1, 2, 3}, {10, 20, 30}};
  std::vector<single_pass_view<int>> columns;
  for (auto& source : sources) {
    columns.push_back(single_pass_view<int>{source});
  }

  auto view = views::transpose(columns);
  ExpectTranspose(view, {{1, 10}, {2, 20}, {3, 30}});
}

TEST(TransposeView, PipeSyntax) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};

  auto view = columns | views::transpose;
  ExpectTranspose(view, {{1, 10}, {2, 20}, {3, 30}});
}

TEST(TransposeView, ModelsInputRange) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};
  auto view = views::transpose(columns);

  static_assert(ranges::input_range<decltype(view)>);

  // Range-v3 algorithms can consume the transpose in a single pass.
  EXPECT_EQ(ranges::distance(view), 3);
}

TEST(TransposeView, ReversedEqualityComparison) {
  // Regression test: comparing the end sentinel against begin (in either
  // operand order) must not dereference the end iterator's null parent.
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};
  auto view = views::transpose(columns);
  EXPECT_NE(view.begin(), view.end());
  EXPECT_NE(view.end(), view.begin());

  std::vector<std::vector<int>> empty;
  auto empty_view = views::transpose(empty);
  EXPECT_EQ(empty_view.begin(), empty_view.end());
  EXPECT_EQ(empty_view.end(), empty_view.begin());
}

TEST(TransposeView, PostIncrementAndLiveIteratorEquality) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};
  auto view = views::transpose(columns);

  // Two live iterators into the same view compare equal (single-pass
  // equivalence semantics; position is not tracked).
  auto it = view.begin();
  EXPECT_EQ(it, view.begin());

  // Post-increment advances the shared per-column iterators before the
  // returned iterator is dereferenced, so it yields the following row.
  EXPECT_EQ(*it++ | ranges::to_vector, (std::vector<int>{2, 20}));
  EXPECT_EQ(*it++ | ranges::to_vector, (std::vector<int>{3, 30}));

  // The shared state is exhausted after the last row.
  ++it;
  EXPECT_EQ(it, view.end());
}

TEST(TransposeView, TransformedSinglePassColumns) {
  // The EKF pattern: a transform producing (prvalue) single-pass views. The
  // transpose materializes them into owned storage at construction, so the
  // captured iterators never outlive the column views.
  std::vector<std::vector<int>> data{{1, 2, 3}, {10, 20, 30}};
  auto view =
      views::transpose(data | ranges::views::transform([](const auto& values) {
                         return single_pass_view<int>{values} | ranges::views::transform([](int x) { return x * 2; });
                       }));
  ExpectTranspose(view, {{2, 20}, {4, 40}, {6, 60}});
}

TEST(TransposeView, EmptyColumns) {
  std::vector<std::vector<int>> columns;

  auto view = views::transpose(columns);
  EXPECT_EQ(view.begin(), view.end());
}

TEST(TransposeView, EmptyFirstColumn) {
  std::vector<std::vector<int>> columns{{}, {1, 2}};

  auto view = views::transpose(columns);
  EXPECT_EQ(view.begin(), view.end());
}

// Materializes the rows of a materialized transposed view and compares them
// with the expected rows, one vector per row.
template <typename View>
void ExpectMaterializedTranspose(View& view, std::vector<std::vector<int>> expected_rows) {
  std::vector<std::vector<int>> rows;
  for (const auto& row : view) {
    rows.push_back(row);
  }
  ASSERT_EQ(rows.size(), expected_rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    EXPECT_EQ(rows[i], expected_rows[i]);
  }
}

TEST(MaterializedTransposeView, MaterializesColumnsIntoRows) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};

  auto view = views::materialized_transpose(columns);
  ExpectMaterializedTranspose(view, {{1, 10}, {2, 20}, {3, 30}});
}

TEST(MaterializedTransposeView, TruncatesToShortestColumn) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20}};  // second is shorter

  auto view = views::materialized_transpose(columns);
  ExpectMaterializedTranspose(view, {{1, 10}, {2, 20}});
}

TEST(MaterializedTransposeView, SinglePassColumns) {
  std::vector<std::vector<int>> sources{{1, 2, 3}, {10, 20, 30}};
  std::vector<single_pass_view<int>> columns;
  for (auto& source : sources) {
    columns.push_back(single_pass_view<int>{source});
  }

  auto view = views::materialized_transpose(columns);
  ExpectMaterializedTranspose(view, {{1, 10}, {2, 20}, {3, 30}});
}

TEST(MaterializedTransposeView, PipeSyntax) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};

  auto view = columns | views::materialized_transpose;
  ExpectMaterializedTranspose(view, {{1, 10}, {2, 20}, {3, 30}});
}

TEST(MaterializedTransposeView, ModelsInputRange) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};
  auto view = views::materialized_transpose(columns);

  static_assert(ranges::input_range<decltype(view)>);

  // Range-v3 algorithms can consume the transpose in a single pass.
  EXPECT_EQ(ranges::distance(view), 3);
}

TEST(MaterializedTransposeView, RowsAreStableWithinLoopBody) {
  // The EKF pattern: the row is used throughout the loop body and must
  // remain valid until the next increment.
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};
  auto view = views::materialized_transpose(columns);

  std::vector<int> first_elements;
  std::vector<int> second_elements;
  for (const auto& row : view) {
    first_elements.push_back(row[0]);
    second_elements.push_back(row[1]);
  }
  EXPECT_EQ(first_elements, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(second_elements, (std::vector<int>{10, 20, 30}));
}

TEST(MaterializedTransposeView, RowsAreForwardRanges) {
  // Unlike transpose_view's lazy rows, materialized rows are owned vectors:
  // they are forward ranges that can be iterated multiple times. With
  // single-pass columns, a lazy row would be consumed by the first
  // distance(); the materialized row is an owned copy and is not.
  std::vector<std::vector<int>> sources{{1, 2, 3}, {10, 20, 30}};
  std::vector<single_pass_view<int>> columns;
  for (auto& source : sources) {
    columns.push_back(single_pass_view<int>{source});
  }
  auto view = views::materialized_transpose(columns);

  auto it = view.begin();
  const auto& row = *it;
  static_assert(ranges::forward_range<decltype(row)>);
  EXPECT_EQ(ranges::distance(row), 2);
  EXPECT_EQ(ranges::distance(row), 2);  // still 2: the row is an owned copy
}

TEST(MaterializedTransposeView, ReusesRowBuffer) {
  // The view materializes each row into a single shared buffer (cache1
  // style): the row address is the same across iterations.
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};
  auto view = views::materialized_transpose(columns);

  const std::vector<int>* first_address = nullptr;
  std::size_t count = 0;
  for (const auto& row : view) {
    if (first_address == nullptr) {
      first_address = &row;
    }
    EXPECT_EQ(&row, first_address);
    ++count;
  }
  EXPECT_EQ(count, 3u);
}

TEST(MaterializedTransposeView, PostIncrementYieldsFollowingRow) {
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};
  auto view = views::materialized_transpose(columns);

  auto it = view.begin();
  // Post-increment advances the shared per-column iterators before the
  // returned iterator is dereferenced, so it yields the following row.
  EXPECT_EQ(*it++, (std::vector<int>{2, 20}));
  EXPECT_EQ(*it, (std::vector<int>{2, 20}));
  ++it;
  EXPECT_EQ(*it, (std::vector<int>{3, 30}));
  ++it;
  EXPECT_EQ(it, view.end());
}

TEST(MaterializedTransposeView, ReversedEqualityComparison) {
  // Regression test: comparing the end sentinel against begin (in either
  // operand order) must not dereference the end iterator's null parent.
  std::vector<std::vector<int>> columns{{1, 2, 3}, {10, 20, 30}};
  auto view = views::materialized_transpose(columns);
  EXPECT_NE(view.begin(), view.end());
  EXPECT_NE(view.end(), view.begin());

  std::vector<std::vector<int>> empty;
  auto empty_view = views::materialized_transpose(empty);
  EXPECT_EQ(empty_view.begin(), empty_view.end());
  EXPECT_EQ(empty_view.end(), empty_view.begin());
}

TEST(MaterializedTransposeView, TransformedSinglePassColumns) {
  // The EKF pattern: a transform producing (prvalue) single-pass views.
  std::vector<std::vector<int>> data{{1, 2, 3}, {10, 20, 30}};
  auto view = views::materialized_transpose(data | ranges::views::transform([](const auto& values) {
                                              return single_pass_view<int>{values} |
                                                     ranges::views::transform([](int x) { return x * 2; });
                                            }));
  ExpectMaterializedTranspose(view, {{2, 20}, {4, 40}, {6, 60}});
}

TEST(MaterializedTransposeView, EmptyColumns) {
  std::vector<std::vector<int>> columns;

  auto view = views::materialized_transpose(columns);
  EXPECT_EQ(view.begin(), view.end());
}

TEST(MaterializedTransposeView, EmptyFirstColumn) {
  std::vector<std::vector<int>> columns{{}, {1, 2}};

  auto view = views::materialized_transpose(columns);
  EXPECT_EQ(view.begin(), view.end());
}

}  // namespace
}  // namespace beluga_estimation
