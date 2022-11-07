#include <execution>

#include <gtest/gtest.h>
#include <range/v3/view.hpp>

#include <beluga/voxel_hash.h>

namespace {

TEST(VoxelHash, Round) {
  using Tuple = std::tuple<double, double>;
  auto hash1 = beluga::voxel_hash<Tuple>{}(std::make_tuple(10.3, 5.0));
  auto hash2 = beluga::voxel_hash<Tuple>{}(std::make_tuple(10.0, 5.0));
  auto hash3 = beluga::voxel_hash<Tuple>{}(std::make_tuple(10.0, 5.3));
  auto hash4 = beluga::voxel_hash<Tuple>{}(std::make_tuple(10.1, 5.1));
  EXPECT_EQ(hash1, hash2);
  EXPECT_EQ(hash2, hash3);
  EXPECT_EQ(hash3, hash4);
  auto hash5 = beluga::voxel_hash<Tuple>{}(std::make_tuple(9.1, 5.1));
  auto hash6 = beluga::voxel_hash<Tuple>{}(std::make_tuple(10.1, 4.1));
  EXPECT_NE(hash1, hash5);
  EXPECT_NE(hash1, hash6);
}

TEST(VoxelHash, VoxelSize) {
  using Tuple = std::tuple<double, double>;
  constexpr double kVoxelSize = 0.1;
  auto hash1 = beluga::voxel_hash<Tuple>{}(std::make_tuple(10.3, 5.13), kVoxelSize);
  auto hash2 = beluga::voxel_hash<Tuple>{}(std::make_tuple(10.33, 5.14), kVoxelSize);
  EXPECT_EQ(hash1, hash2);
  auto hash3 = beluga::voxel_hash<Tuple>{}(std::make_tuple(10.0, 5.0), kVoxelSize);
  EXPECT_NE(hash1, hash3);
}

TEST(VoxelHash, Negative) {
  using Tuple = std::tuple<double, double>;
  constexpr double kVoxelSize = 0.1;
  auto hash1 = beluga::voxel_hash<Tuple>{}(std::make_tuple(-10.3, -2.13), kVoxelSize);
  auto hash2 = beluga::voxel_hash<Tuple>{}(std::make_tuple(-10.27, -2.14), kVoxelSize);
  EXPECT_EQ(hash1, hash2);
  auto hash3 = beluga::voxel_hash<Tuple>{}(std::make_tuple(-10.0, -2.0), kVoxelSize);
  EXPECT_NE(hash1, hash3);
}

TEST(VoxelHash, NoCollisions) {
  using Tuple = std::tuple<double, double, double>;
  constexpr int kLimit = 100;

  // Brute-force search for collisions.
  // With kLimit = 100 and a 3-element tuple, we test 8'120'601 combinations.
  auto hashes = ranges::views::cartesian_product(
                    ranges::views::closed_iota(-kLimit, kLimit), ranges::views::closed_iota(-kLimit, kLimit),
                    ranges::views::closed_iota(-kLimit, kLimit)) |
                ranges::views::transform([](const auto& tuple) { return beluga::voxel_hash<Tuple>{}(tuple); }) |
                ranges::to<std::vector>;

  std::sort(std::execution::par_unseq, std::begin(hashes), std::end(hashes));
  auto it = std::adjacent_find(std::execution::par_unseq, std::begin(hashes), std::end(hashes));
  bool has_duplicates = it != std::end(hashes);
  ASSERT_FALSE(has_duplicates);
}

TEST(VoxelHash, Array) {
  using Array = std::array<double, 3>;
  using Tuple = std::tuple<double, double, double>;
  auto hash1 = beluga::voxel_hash<Array>{}({1., 2., 3.});
  auto hash2 = beluga::voxel_hash<Tuple>{}({1., 2., 3.});
  EXPECT_EQ(hash1, hash2);
}

}  // namespace
