// Copyright 2022 Ekumen, Inc.
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

#include <beluga/algorithm/particle_filter.h>

namespace beluga {

template <>
struct spatial_hash<double, void> {
 public:
  constexpr std::size_t operator()(double value, double resolution = 1.) const {
    return spatial_hash<std::tuple<double>>{}(std::make_tuple(value), resolution);
  }
};

}  // namespace beluga

namespace {

template <class Mixin>
class MockMotionModel : public Mixin {
 public:
  template <class... Args>
  explicit MockMotionModel(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] double apply_motion(double state) { return state; }
};

template <class Mixin>
class MockSensorModel : public Mixin {
 public:
  template <class... Args>
  explicit MockSensorModel(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] double importance_weight(double) { return 1.; }

  template <class Generator>
  [[nodiscard]] double generate_random_state(Generator&) {
    return 0.;
  }
};

TEST(MCL, InitializeFilter) {
  auto filter =
      beluga::MCL<MockMotionModel, MockSensorModel, double>{beluga::FixedResamplingParam{.max_samples = 1'000}};
  ASSERT_EQ(filter.particles().size(), 1'000);
}

TEST(AMCL, InitializeFilter) {
  auto filter = beluga::AMCL<MockMotionModel, MockSensorModel, double>{
      beluga::AdaptiveGenerationParam{.alpha_slow = 0.001, .alpha_fast = 0.1},
      beluga::KldResamplingParam{
          .min_samples = 1'000, .max_samples = 2'000, .spatial_resolution = 1., .kld_epsilon = 0.05, .kld_z = 3.}};
  ASSERT_GE(filter.particles().size(), 1'000);
}

}  // namespace
