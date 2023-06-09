// Copyright 2023 Ekumen, Inc.
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

#include "beluga/algorithm/raycasting/bresenham.hpp"

#include <vector>

#include <Eigen/Core>

#include <gmock/gmock.h>

#include <range/v3/range/conversion.hpp>

namespace beluga {

TEST(Bresenham, MultiPassGuarantee) {
  // See comparison between iterators in
  // https://en.cppreference.com/w/cpp/iterator/forward_iterator
  const auto algorithm = Bresenham2i{Bresenham2i::kStandard};

  const auto line = algorithm({0, 0}, {5, 5});
  for (auto it1 = line.begin(), it2 = line.begin(); it1 != line.end(); ++it1, ++it2) {
    ASSERT_EQ(it1, it2);
  }

  // Test that iterating over the sequence using an independent
  // iterator doesn't change what an existing iterator refers to.
  auto it3 = line.begin();
  ++it3;
  auto it4 = it3;
  const auto expected_value = *it3;
  ++it4;
  ASSERT_EQ(expected_value, *it3);
}

TEST(Bresenham, Standard) {
  auto algorithm = Bresenham2i{Bresenham2i::kStandard};

  {
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}};
    const auto trace = algorithm({0, 0}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // |   | > |
    // +---+---+
    // | > |   |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}, {1, 1}};
    const auto trace = algorithm({0, 0}, {1, 1}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // |   | < |
    // +---+---+
    // | < |   |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{1, 1}, {0, 0}};
    const auto trace = algorithm({1, 1}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+
    // |   |   | > |
    // +---+---+---+
    // | > | > |   |
    // +---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}, {1, 0}, {2, 1}};
    const auto trace = algorithm({0, 0}, {2, 1}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+
    // |   | < | < |
    // +---+---+---+
    // | < |   |   |
    // +---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{2, 1}, {1, 1}, {0, 0}};
    const auto trace = algorithm({2, 1}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // | v |   |
    // +---+---+
    // | v |   |
    // +---+---+
    // | v |   |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 2}, {0, 1}, {0, 0}};
    const auto trace = algorithm({0, 2}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+---+
    // |   |   |   | < |
    // +---+---+---+---+
    // |   | < | < |   |
    // +---+---+---+---+
    // | < |   |   |   |
    // +---+---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{3, 2}, {2, 1}, {1, 1}, {0, 0}};
    const auto trace = algorithm({3, 2}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }
}

TEST(Bresenham, Modified) {
  auto algorithm = Bresenham2i{Bresenham2i::kModified};

  {
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}};
    const auto trace = algorithm({0, 0}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // | > | > |
    // +---+---+
    // | > | > |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}, {1, 0}, {0, 1}, {1, 1}};
    const auto trace = algorithm({0, 0}, {1, 1}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // | < | < |
    // +---+---+
    // | < | < |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{1, 1}, {0, 1}, {1, 0}, {0, 0}};
    const auto trace = algorithm({1, 1}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+
    // |   | > | > |
    // +---+---+---+
    // | > | > |   |
    // +---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 0}, {1, 0}, {1, 1}, {2, 1}};
    const auto trace = algorithm({0, 0}, {2, 1}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+
    // |   | < | < |
    // +---+---+---+
    // | < | < |   |
    // +---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{2, 1}, {1, 1}, {1, 0}, {0, 0}};
    const auto trace = algorithm({2, 1}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+
    // | v |   |
    // +---+---+
    // | v |   |
    // +---+---+
    // | v |   |
    // +---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{0, 2}, {0, 1}, {0, 0}};
    const auto trace = algorithm({0, 2}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }

  {
    // +---+---+---+---+
    // |   |   | < | < |
    // +---+---+---+---+
    // |   | < | < |   |
    // +---+---+---+---+
    // | < | < |   |   |
    // +---+---+---+---+
    const auto expected_trace = std::vector<Eigen::Vector2i>{{3, 2}, {2, 2}, {2, 1}, {1, 1}, {1, 0}, {0, 0}};
    const auto trace = algorithm({3, 2}, {0, 0}) | ranges::to<std::vector>;
    EXPECT_EQ(trace, expected_trace);
  }
}

}  // namespace beluga
