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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "beluga/sensor/data/dense_grid.hpp"

namespace {

template <std::size_t W, std::size_t H>
class Image : public beluga::BaseDenseGrid2<Image<W, H>> {
 public:
  using index_type = std::pair<std::size_t, std::size_t>;
  using value_type = uint8_t;

  Image() = default;

  explicit Image(std::array<std::array<value_type, W>, H> data) : data_(std::move(data)) {}

  [[nodiscard]] index_type index_at(int xi, int yi) const {
    return index_type{static_cast<std::size_t>(xi), static_cast<std::size_t>(yi)};
  }

  [[nodiscard]] index_type index_at(const Eigen::Vector2i& pi) const { return index_at(pi.x(), pi.y()); }

  using beluga::BaseDenseGrid2<Image<W, H>>::data_at;

  [[nodiscard]] std::optional<value_type> data_at(const index_type& index) const {
    return index.first < W && index.second < H ? std::make_optional(data_[index.first][index.second]) : std::nullopt;
  }

  [[nodiscard]] std::size_t width() const { return W; }
  [[nodiscard]] std::size_t height() const { return H; }
  [[nodiscard]] double resolution() const { return 1.; }

 private:
  std::array<std::array<value_type, W>, H> data_;
};

TEST(DenseGrid2, Limits) {
  const auto grid = Image<5, 5>{};

  EXPECT_TRUE(grid.contains(0, 0));
  EXPECT_FALSE(grid.contains(-1, 0));
  EXPECT_FALSE(grid.contains(0, -1));
  EXPECT_TRUE(grid.contains(2, 2));
  EXPECT_FALSE(grid.contains(5, 0));
  EXPECT_FALSE(grid.contains(0, 5));

  EXPECT_TRUE(grid.contains(Eigen::Vector2i(0, 0)));
  EXPECT_FALSE(grid.contains(Eigen::Vector2i(-1, 0)));
  EXPECT_FALSE(grid.contains(Eigen::Vector2i(0, -1)));
  EXPECT_TRUE(grid.contains(Eigen::Vector2i(2, 2)));
  EXPECT_FALSE(grid.contains(Eigen::Vector2i(5, 0)));
  EXPECT_FALSE(grid.contains(Eigen::Vector2i(0, 5)));
}

TEST(DenseGrid2, Data) {
  const auto grid =
      Image<5, 5>({{{0, 0, 0, 0, 0}, {0, 1, 1, 1, 0}, {0, 1, 2, 1, 0}, {0, 1, 1, 1, 0}, {0, 0, 0, 0, 0}}});

  EXPECT_EQ(grid.data_at(0, 0), 0);
  EXPECT_EQ(grid.data_at(2, 2), 2);
  EXPECT_EQ(grid.data_at(3, 1), 1);
  EXPECT_EQ(grid.data_at(-1, 0), std::nullopt);
  EXPECT_EQ(grid.data_at(0, -1), std::nullopt);
  EXPECT_EQ(grid.data_at(5, 0), std::nullopt);
  EXPECT_EQ(grid.data_at(0, 5), std::nullopt);

  EXPECT_EQ(grid.data_at(Eigen::Vector2i(0, 0)), 0);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(2, 2)), 2);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(3, 1)), 1);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(-1, 0)), std::nullopt);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(0, -1)), std::nullopt);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(5, 0)), std::nullopt);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(0, 5)), std::nullopt);
}

TEST(DenseGrid2, NearestData) {
  const auto grid =
      Image<5, 5>({{{0, 0, 0, 0, 0}, {0, 1, 1, 1, 0}, {0, 1, 2, 1, 0}, {0, 1, 1, 1, 0}, {0, 0, 0, 0, 0}}});

  EXPECT_EQ(grid.data_near(0.8, 0.2), 0);
  EXPECT_EQ(grid.data_near(2.1, 2.9), 2);
  EXPECT_EQ(grid.data_near(3.5, 1.5), 1);
  EXPECT_EQ(grid.data_near(-0.1, 0), std::nullopt);
  EXPECT_EQ(grid.data_near(0, -0.1), std::nullopt);
  EXPECT_EQ(grid.data_near(5.25, 0), std::nullopt);
  EXPECT_EQ(grid.data_near(0, 5.25), std::nullopt);

  EXPECT_EQ(grid.data_near(Eigen::Vector2d(0.8, 0.2)), 0);
  EXPECT_EQ(grid.data_near(Eigen::Vector2d(2.1, 2.9)), 2);
  EXPECT_EQ(grid.data_near(Eigen::Vector2d(3.5, 1.5)), 1);
  EXPECT_EQ(grid.data_near(Eigen::Vector2d(-0.1, 0)), std::nullopt);
  EXPECT_EQ(grid.data_near(Eigen::Vector2d(0, -0.1)), std::nullopt);
  EXPECT_EQ(grid.data_near(Eigen::Vector2d(5.25, 0)), std::nullopt);
  EXPECT_EQ(grid.data_near(Eigen::Vector2d(0, 5.25)), std::nullopt);
}

TEST(DenseGrid2, Neighborhood4) {
  const auto grid = Image<5, 5>{};

  {
    const auto expected_neighborhood =
        std::vector{Eigen::Vector2i{3, 2}, Eigen::Vector2i{2, 3}, Eigen::Vector2i{1, 2}, Eigen::Vector2i{2, 1}};
    ASSERT_THAT(grid.neighborhood4(2, 2), testing::Pointwise(testing::Eq(), expected_neighborhood));
    ASSERT_THAT(grid.neighborhood4(Eigen::Vector2i(2, 2)), testing::Pointwise(testing::Eq(), expected_neighborhood));
  }

  {
    const auto expected_neighborhood = std::vector{Eigen::Vector2i{1, 0}, Eigen::Vector2i{0, 1}};
    ASSERT_THAT(grid.neighborhood4(0, 0), testing::Pointwise(testing::Eq(), expected_neighborhood));
    ASSERT_THAT(grid.neighborhood4(Eigen::Vector2i(0, 0)), testing::Pointwise(testing::Eq(), expected_neighborhood));
  }

  {
    const auto expected_neighborhood = std::vector{Eigen::Vector2i{3, 4}, Eigen::Vector2i{4, 3}};
    ASSERT_THAT(grid.neighborhood4(4, 4), testing::Pointwise(testing::Eq(), expected_neighborhood));
    ASSERT_THAT(grid.neighborhood4(Eigen::Vector2i(4, 4)), testing::Pointwise(testing::Eq(), expected_neighborhood));
  }
}

}  // namespace
