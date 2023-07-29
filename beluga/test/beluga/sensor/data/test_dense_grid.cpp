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

#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "beluga/sensor/data/dense_grid2_mixin.hpp"
#include "beluga/sensor/data/regular_grid2_mixin.hpp"

#include <range/v3/range/conversion.hpp>

#include <Eigen/Core>

#include <ciabatta/ciabatta.hpp>

namespace {

template <std::size_t W, std::size_t H>
struct ImageSize {
  static constexpr std::size_t Width = W;
  static constexpr std::size_t Height = H;
};

template <typename Mixin, typename ImageSize>
class ImageMixin : public Mixin {
 public:
  using value_type = uint8_t;
  using data_storage = std::array<std::array<value_type, ImageSize::Width>, ImageSize::Height>;

  template <typename... Args>
  explicit ImageMixin(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  template <typename... Args>
  explicit ImageMixin(data_storage data, Args&&... args) : Mixin(std::forward<Args>(args)...), data_(std::move(data)) {}

  using Mixin::data_at;
  [[nodiscard]] std::optional<value_type> data_at(int xi, int yi) const {
    return xi < ImageSize::Width && yi < ImageSize::Height ? std::make_optional(data_[xi][yi]) : std::nullopt;
  }

  [[nodiscard]] std::size_t width() const { return ImageSize::Width; }
  [[nodiscard]] std::size_t height() const { return ImageSize::Height; }
  [[nodiscard]] double resolution() const { return 1.; }

 private:
  std::array<std::array<value_type, ImageSize::Width>, ImageSize::Height> data_;
};

template <std::size_t W, std::size_t H>
using ImageCombined = ciabatta::mixin<
    ciabatta::curry<ImageMixin, ImageSize<W, H>>::template mixin,
    beluga::DenseGrid2Mixin,
    beluga::RegularGrid2Mixin>;

template <std::size_t W, std::size_t H>
struct Image : public ImageCombined<W, H> {
  template <typename... Args>
  explicit Image(Args&&... args) : ImageCombined<W, H>(std::forward<Args>(args)...) {}
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
  const auto grid = Image<5, 5>(
      Image<5, 5>::data_storage{{{0, 0, 0, 0, 0}, {0, 1, 1, 1, 0}, {0, 1, 2, 1, 0}, {0, 1, 1, 1, 0}, {0, 0, 0, 0, 0}}});

  EXPECT_EQ(grid.data_at(Eigen::Vector2i(0, 0)), 0);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(2, 2)), 2);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(3, 1)), 1);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(-1, 0)), std::nullopt);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(0, -1)), std::nullopt);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(5, 0)), std::nullopt);
  EXPECT_EQ(grid.data_at(Eigen::Vector2i(0, 5)), std::nullopt);
}

TEST(DenseGrid2, NearestData) {
  const auto grid = Image<5, 5>(
      Image<5, 5>::data_storage{{{0, 0, 0, 0, 0}, {0, 1, 1, 1, 0}, {0, 1, 2, 1, 0}, {0, 1, 1, 1, 0}, {0, 0, 0, 0, 0}}});

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
