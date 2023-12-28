// Copyright 2022-2023 Ekumen, Inc.
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

#include <gtest/gtest.h>

#include <beluga/algorithm/spatial_hash.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/cartesian_product.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>

namespace {

TEST(SpatialHash, Round) {
  using Tuple = std::tuple<double, double>;
  constexpr std::array kClusteringResolution{1., 1.};
  auto hasher = beluga::spatial_hash<Tuple>{kClusteringResolution};
  auto hash1 = hasher(std::make_tuple(10.3, 5.0));
  auto hash2 = hasher(std::make_tuple(10.0, 5.0));
  auto hash3 = hasher(std::make_tuple(10.0, 5.3));
  auto hash4 = hasher(std::make_tuple(10.1, 5.1));
  EXPECT_EQ(hash1, hash2);
  EXPECT_EQ(hash2, hash3);
  EXPECT_EQ(hash3, hash4);
  auto hash5 = hasher(std::make_tuple(9.1, 5.1));
  auto hash6 = hasher(std::make_tuple(10.1, 4.1));
  EXPECT_NE(hash1, hash5);
  EXPECT_NE(hash1, hash6);
}

TEST(SpatialHash, Resolution) {
  using Tuple = std::tuple<double, double>;
  constexpr std::array kClusteringResolution{0.1, 0.1};
  auto hasher = beluga::spatial_hash<Tuple>{kClusteringResolution};

  auto hash1 = hasher(std::make_tuple(10.3, 5.13));
  auto hash2 = hasher(std::make_tuple(10.33, 5.14));
  EXPECT_EQ(hash1, hash2);
  auto hash3 = hasher(std::make_tuple(10.0, 5.0));
  EXPECT_NE(hash1, hash3);
}

TEST(SpatialHash, Negative) {
  using Tuple = std::tuple<double, double>;
  constexpr std::array kClusteringResolution{0.1, 0.1};
  auto hasher = beluga::spatial_hash<Tuple>{kClusteringResolution};
  auto hash1 = hasher(std::make_tuple(-10.3, -2.13));
  auto hash2 = hasher(std::make_tuple(-10.27, -2.14));
  EXPECT_EQ(hash1, hash2);
  auto hash3 = hasher(std::make_tuple(-10.0, -2.0));
  EXPECT_NE(hash1, hash3);
}

TEST(SpatialHash, NoCollisions) {
  using Tuple = std::tuple<double, double, double>;
  constexpr int kLimit = 100;
  constexpr std::array kClusteringResolution{1., 1., 1.};
  auto hasher = beluga::spatial_hash<Tuple>{kClusteringResolution};

  // Brute-force search for collisions.
  // With kLimit = 100 and a 3-element tuple, we test 8'120'601 combinations.
  auto hashes = ranges::views::cartesian_product(
                    ranges::views::closed_iota(-kLimit, kLimit), ranges::views::closed_iota(-kLimit, kLimit),
                    ranges::views::closed_iota(-kLimit, kLimit)) |
                ranges::views::transform([hasher](const auto& tuple) {
                  return hasher(std::make_tuple(
                      static_cast<double>(std::get<0>(tuple)), static_cast<double>(std::get<1>(tuple)),
                      static_cast<double>(std::get<2>(tuple))));
                }) |
                ranges::to<std::vector>;

  std::sort(std::begin(hashes), std::end(hashes));
  auto it = std::adjacent_find(std::begin(hashes), std::end(hashes));
  bool has_duplicates = it != std::end(hashes);
  ASSERT_FALSE(has_duplicates);
}

TEST(SpatialHash, Array) {
  using Array = std::array<double, 3>;
  using Tuple = std::tuple<double, double, double>;
  constexpr std::array kClusteringResolution{1., 1., 1.};
  auto array_hasher = beluga::spatial_hash<Array>{kClusteringResolution};
  auto tuple_hasher = beluga::spatial_hash<Tuple>{kClusteringResolution};

  auto hash1 = array_hasher({1., 2., 3.});
  auto hash2 = tuple_hasher({1., 2., 3.});
  EXPECT_EQ(hash1, hash2);
}

TEST(SpatialHash, NonPeriodicityCheck1) {
  using Array = std::array<double, 8>;
  constexpr std::array kClusteringResolution{1., 1., 1., 1., 1., 1., 1., 1.};

  auto uut = beluga::spatial_hash<Array>{kClusteringResolution};

  auto hash0 = uut({0., 0., 0., 0., 0., 0., 0., 0.});
  auto hash1 = uut({64., 0., 0., 0., 0., 0., 0., 0.});
  auto hash2 = uut({128., 0., 0., 0., 0., 0., 0., 0.});
  auto hash3 = uut({192., 0., 0., 0., 0., 0., 0., 0.});
  auto hash4 = uut({256., 0., 0., 0., 0., 0., 0., 0.});

  EXPECT_NE(hash0, hash1);
  EXPECT_NE(hash0, hash2);
  EXPECT_NE(hash0, hash3);
  EXPECT_NE(hash0, hash4);
}

TEST(SpatialHash, NonPeriodicityCheck2) {
  using Array = std::array<double, 8>;
  constexpr std::array kClusteringResolution{1., 1., 1., 1., 1., 1., 1., 1.};
  constexpr double kStep = 2.0;

  auto uut = beluga::spatial_hash<Array>{kClusteringResolution};

  auto ref_hash = uut({0., 0., 0., 0., 0., 0., 0., 0.});

  for (double n = kStep; n < 1024.; n += kStep) {
    auto hash0 = uut({n, 0., 0., 0., 0., 0., 0., 0.});
    auto hash1 = uut({0., n, 0., 0., 0., 0., 0., 0.});
    auto hash2 = uut({0., 0., n, 0., 0., 0., 0., 0.});
    auto hash3 = uut({0., 0., 0., n, 0., 0., 0., 0.});
    auto hash4 = uut({0., 0., 0., 0., n, 0., 0., 0.});
    auto hash5 = uut({0., 0., 0., 0., 0., n, 0., 0.});
    auto hash6 = uut({0., 0., 0., 0., 0., 0., n, 0.});
    auto hash7 = uut({0., 0., 0., 0., 0., 0., 0., n});
    EXPECT_NE(ref_hash, hash0);
    EXPECT_NE(ref_hash, hash1);
    EXPECT_NE(ref_hash, hash2);
    EXPECT_NE(ref_hash, hash3);
    EXPECT_NE(ref_hash, hash4);
    EXPECT_NE(ref_hash, hash5);
    EXPECT_NE(ref_hash, hash6);
    EXPECT_NE(ref_hash, hash7);
  }
}

}  // namespace
