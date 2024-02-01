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

#include <sophus/se3.hpp>

#include <range/v3/range/conversion.hpp>

#include <beluga/sensor/data/laser_scan.hpp>
#include <beluga/testing.hpp>

namespace {

using beluga::testing::Vector2Near;
using testing::ElementsAre;

constexpr double kMinRange = 5.;
constexpr double kMaxRange = 100.;

class SimpleLaserScan : public beluga::BaseLaserScan<SimpleLaserScan> {
 public:
  using Scalar = double;

  SimpleLaserScan(std::vector<double> ranges, std::vector<double> angles, Sophus::SE3d origin = Sophus::SE3d{})
      : ranges_{std::move(ranges)}, angles_{std::move(angles)}, origin_{std::move(origin)} {}

  [[nodiscard]] const auto& origin() const { return origin_; }

  [[nodiscard]] auto ranges() const { return ranges_ | ranges::views::all; }

  [[nodiscard]] auto angles() const { return angles_ | ranges::views::all; }

  [[nodiscard]] static auto min_range() { return kMinRange; }

  [[nodiscard]] static auto max_range() { return kMaxRange; }

 private:
  std::vector<double> ranges_;
  std::vector<double> angles_;
  Sophus::SE3d origin_;
};

TEST(LaserScan, InvalidValues) {
  constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
  auto laser_scan = SimpleLaserScan{
      {120., 0., 50., kNan},
      {0.1, 0.2, 0.3, 0.4},
  };
  auto polar = laser_scan.points_in_polar_coordinates() | ranges::to<std::vector>;
  ASSERT_EQ(polar.size(), 1UL);
  ASSERT_THAT(polar.front(), Vector2Near<double>({50., 0.3}, 0.001));
}

TEST(LaserScan, TransformPolar) {
  const auto pi = Sophus::Constants<double>::pi();
  auto laser_scan = SimpleLaserScan{
      {10., 20., 30., 40.},
      {0., pi / 2, pi, -pi / 2},
  };
  auto polar = laser_scan.points_in_polar_coordinates() | ranges::to<std::vector>;
  ASSERT_THAT(
      polar, ElementsAre(
                 Vector2Near<double>({10., 0.}, 0.001),      //
                 Vector2Near<double>({20., pi / 2}, 0.001),  //
                 Vector2Near<double>({30., pi}, 0.001),      //
                 Vector2Near<double>({40., -pi / 2}, 0.001)));
}

TEST(LaserScan, TransformCartesian) {
  const auto pi = Sophus::Constants<double>::pi();
  auto laser_scan = SimpleLaserScan{
      {10., 20., 30., 40.},
      {0., pi / 2, pi, -pi / 2},
  };
  auto polar = laser_scan.points_in_cartesian_coordinates() | ranges::to<std::vector>;
  ASSERT_THAT(
      polar, ElementsAre(
                 Vector2Near<double>({10., 0.}, 0.001),   //
                 Vector2Near<double>({0., 20.}, 0.001),   //
                 Vector2Near<double>({-30., 0.}, 0.001),  //
                 Vector2Near<double>({0., -40.}, 0.001)));
}

}  // namespace
