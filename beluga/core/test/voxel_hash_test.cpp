#include <execution>

#include <gtest/gtest.h>
#include <range/v3/view.hpp>

#include <beluga/voxel_hash.h>

namespace {

TEST(VoxelHash, NoCollisions) {
  using Tuple = std::tuple<double, double, double>;
  constexpr int kLimit = 100;

  // Brute-force search for collisions
  // With kLimit = 100 and a 3-element tuple, we test 8'120'601 combinations
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

}  // namespace
