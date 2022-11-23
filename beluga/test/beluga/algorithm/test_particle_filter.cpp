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

#include <gmock/gmock.h>

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

template <template <class> class Mixin>
class MockMixin : public ciabatta::mixin<MockMixin<Mixin>, Mixin> {
 public:
  using ciabatta::mixin<MockMixin<Mixin>, Mixin>::mixin;
  MOCK_METHOD(double, apply_motion, (double state));
  MOCK_METHOD(double, importance_weight, (double state));

  std::size_t max_samples() { return 10.; }

  template <class Particle>
  auto generate_samples() {
    return ranges::views::generate([]() { return std::make_pair(1., 1.); });
  }

  template <class Range>
  auto generate_samples_from(Range&& range) {
    return range | ranges::views::all;
  }

  auto take_samples() { return ranges::views::take_exactly(max_samples()); }
};

template <class Mixin>
using ParticleFilter = typename beluga::BootstrapParticleFilter<Mixin, std::vector<std::pair<double, double>>>;

TEST(BootstrapParticleFilter, Initialize) {
  auto filter = MockMixin<ParticleFilter>();
  ASSERT_EQ(filter.particles().size(), 10);
}

TEST(BootstrapParticleFilter, Update) {
  auto filter = MockMixin<ParticleFilter>();
  EXPECT_CALL(filter, apply_motion(1.)).WillRepeatedly(testing::Return(2.));
  EXPECT_CALL(filter, importance_weight(2.)).WillRepeatedly(testing::Return(3.));
  filter.update();
  for (auto [state, weight] : filter.particles()) {
    ASSERT_EQ(state, 2.);
    ASSERT_EQ(weight, 3.);
  }
}

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
  constexpr std::size_t kMaxSamples = 1'000;
  auto filter = beluga::MCL<MockMotionModel, MockSensorModel, double>{beluga::FixedResamplingParam{kMaxSamples}};
  ASSERT_EQ(filter.particles().size(), kMaxSamples);
}

TEST(AMCL, InitializeFilter) {
  constexpr double kAlphaSlow = 0.001;
  constexpr double kAlphaFast = 0.1;
  constexpr std::size_t kMinSamples = 2'000;
  constexpr std::size_t kMaxSamples = 2'000;
  constexpr double kSpatialResolution = 1.;
  constexpr double kKldEpsilon = 0.05;
  constexpr double kKldZ = 3.;
  auto filter = beluga::AMCL<MockMotionModel, MockSensorModel, double>{
      beluga::AdaptiveGenerationParam{kAlphaSlow, kAlphaFast},
      beluga::KldResamplingParam{kMinSamples, kMaxSamples, kSpatialResolution, kKldEpsilon, kKldZ}};
  ASSERT_GE(filter.particles().size(), kMinSamples);
}

}  // namespace
