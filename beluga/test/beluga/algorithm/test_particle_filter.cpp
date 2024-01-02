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

#include <gmock/gmock.h>

#include <execution>

#include <beluga/algorithm/particle_filter.hpp>
#include <beluga/views.hpp>

#include <ciabatta/ciabatta.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>

namespace {

using testing::Each;
using testing::Return;
using testing::ReturnPointee;

template <class Mixin>
class MockMixin : public Mixin {
 public:
  MOCK_METHOD(double, apply_motion, (double state), (const));
  MOCK_METHOD(double, importance_weight, (double state), (const));
  MOCK_METHOD(bool, do_resampling_vote, ());

  auto particles() { return particles_ | ranges::views::common; }

  auto states() { return particles_ | beluga::views::elements<0>; }

  auto weights() { return particles_ | beluga::views::elements<1>; }

  template <class Range>
  void initialize_particles(Range&& input) {
    particles_ = input |                                                                              //
                 ranges::views::transform([](double state) { return std::make_pair(state, 1.0); }) |  //
                 ranges::to<std::vector>;
  }

  template <class Generator>
  auto generate_samples(Generator&&) const {
    return ranges::views::generate([]() { return 0.0; });
  }

  template <class Generator>
  auto generate_samples_from_particles(Generator&&) const {
    // for the purpose of the test, return the existing states unchanged
    return particles_ | beluga::views::elements<0> | ranges::views::common;
  }

  auto take_samples() const { return ranges::views::take_exactly(10); }

  template <class Generator>
  double apply_motion(double state, Generator&) const {
    return apply_motion(state);
  }

 private:
  std::vector<std::pair<double, double>> particles_;
};

using ParticleFilter = ciabatta::
    mixin<beluga::BootstrapParticleFilter, MockMixin, ciabatta::provides<beluga::BaseParticleFilterInterface>::mixin>;

TEST(BootstrapParticleFilter, InitializeParticles) {
  auto filter = ParticleFilter{};
  ASSERT_EQ(filter.states().size(), 10);
  EXPECT_THAT(filter.states() | ranges::to<std::vector>, Each(0.));
  EXPECT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.));
}

enum class InterfaceType { kNoParam, kSequential, kParallel };

struct BootstrapParticleFilterOperationLoopTest : public testing::TestWithParam<InterfaceType> {
  void SetUp() override {
    switch (GetParam()) {
      case InterfaceType::kNoParam:
        sample = [](auto& filter) { filter.sample(); };
        reweight = [](auto& filter) { filter.reweight(); };
        break;
      case InterfaceType::kSequential:
        sample = [](auto& filter) { filter.sample(std::execution::seq); };
        reweight = [](auto& filter) { filter.reweight(std::execution::seq); };
        break;
      case InterfaceType::kParallel:
        sample = [](auto& filter) { filter.sample(std::execution::par); };
        reweight = [](auto& filter) { filter.reweight(std::execution::par); };
        break;
    }
    resample = [](auto& filter) { filter.resample(); };
  }

  std::function<void(ParticleFilter&)> sample;
  std::function<void(ParticleFilter&)> reweight;
  std::function<void(ParticleFilter&)> resample;
};

TEST_P(BootstrapParticleFilterOperationLoopTest, UpdateWithoutResampling) {
  auto filter = ParticleFilter{};

  const auto motion_increment = 2.0;
  const auto weight_reduction_factor = 0.707;

  auto expected_initial_state = 0.0;
  auto expected_final_state = motion_increment;

  EXPECT_CALL(filter, apply_motion(::testing::_)).WillRepeatedly(ReturnPointee(&expected_final_state));
  EXPECT_CALL(filter, importance_weight(::testing::_)).WillRepeatedly(Return(weight_reduction_factor));
  EXPECT_CALL(filter, do_resampling_vote()).WillRepeatedly(Return(false));

  for (auto iteration = 0; iteration < 5; ++iteration) {
    // at the start of the iteration weights are max normalized
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_initial_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.0));

    // apply motion on all particles; particles will have updated states, but unchanged weights
    sample(filter);
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_final_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.0));

    // updating and renormalizing particle weights, states remain unchanged
    reweight(filter);
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_final_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.0));

    // resample, but with sampling policy preventing decimation. Particle weights will be renormalized.
    resample(filter);
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.0));

    expected_initial_state = expected_final_state;
    expected_final_state += motion_increment;
  }
}

TEST_P(BootstrapParticleFilterOperationLoopTest, UpdateWithResampling) {
  auto filter = ParticleFilter{};

  const auto motion_increment = 2.0;
  const auto weight_reduction_factor = 0.707;

  auto expected_initial_state = 0.0;
  auto expected_final_state = motion_increment;

  EXPECT_CALL(filter, apply_motion(::testing::_)).WillRepeatedly(ReturnPointee(&expected_final_state));
  EXPECT_CALL(filter, importance_weight(::testing::_)).WillRepeatedly(Return(weight_reduction_factor));
  EXPECT_CALL(filter, do_resampling_vote()).WillRepeatedly(Return(true));

  for (auto iteration = 0; iteration < 4; ++iteration) {
    // at the start of the iteration weights are max normalized
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_initial_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.0));

    // apply motion on all particles, particles will have updated states, but unchanged weights
    sample(filter);
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_final_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.0));

    // updating and renormalizing particle weights, states remain unchanged
    reweight(filter);
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_final_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.0));

    // resample, resampling will reset weights to 1.0 (therefore keeping them max normalized)
    resample(filter);
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.0));

    expected_initial_state = expected_final_state;
    expected_final_state += motion_increment;
  }
}

INSTANTIATE_TEST_SUITE_P(
    BootstrapParticleFilterOperationLoopTestSuite,
    BootstrapParticleFilterOperationLoopTest,
    testing::Values(InterfaceType::kNoParam, InterfaceType::kSequential, InterfaceType::kParallel));

}  // namespace
