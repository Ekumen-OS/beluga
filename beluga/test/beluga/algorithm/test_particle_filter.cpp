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

#include <beluga/algorithm/particle_filter.hpp>
#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views.hpp>

#include <ciabatta/ciabatta.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>

namespace beluga {
template <>
struct particle_traits<std::pair<double, double>> {
  template <typename T>
  static constexpr auto state(T&& particle) {
    return particle.first;
  }

  template <typename T>
  static constexpr auto weight(T&& particle) {
    return particle.second;
  }
};
}  // namespace beluga

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
    particles_ = input | ranges::to<std::vector>;
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

  auto take_samples() const {
    return ranges::views::transform([](double state) { return std::make_pair(state, 1.0); }) |
           ranges::views::take_exactly(10);
  }

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

TEST(BootstrapParticleFilter, UpdateWithoutResampling) {
  auto filter = ParticleFilter{};

  const auto motion_increment = 2.0;
  const auto weight_reduction_factor = 0.707;

  auto expected_initial_state = 0.0;
  auto expected_final_state = motion_increment;
  auto expected_initial_weight = 1.0;
  auto expected_final_weight = weight_reduction_factor;

  EXPECT_CALL(filter, apply_motion(::testing::_)).WillRepeatedly(ReturnPointee(&expected_final_state));
  EXPECT_CALL(filter, importance_weight(::testing::_)).WillRepeatedly(Return(weight_reduction_factor));
  EXPECT_CALL(filter, do_resampling_vote()).WillRepeatedly(Return(false));

  for (auto iteration = 0; iteration < 5; ++iteration) {
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_initial_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(expected_initial_weight));

    // apply motion on all particles, particles will have updated states, but unchanged weight
    filter.sample();
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_final_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(expected_initial_weight));

    // updating particle weights, particles will have updated weights, but unchanged state
    filter.reweight();
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_final_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(expected_final_weight));

    // resample, but with sampling policy preventing decimation
    filter.resample();

    expected_initial_state = expected_final_state;
    expected_initial_weight = expected_final_weight;
    expected_final_state += motion_increment;
    expected_final_weight *= weight_reduction_factor;
  }
}

TEST(BootstrapParticleFilter, UpdateWithResampling) {
  auto filter = ParticleFilter{};

  const auto motion_increment = 2.0;
  const auto weight_reduction_factor = 0.707;

  auto expected_initial_state = 0.0;
  auto expected_final_state = motion_increment;
  const auto expected_initial_weight = 1.0;
  const auto expected_final_weight = weight_reduction_factor;

  EXPECT_CALL(filter, apply_motion(::testing::_)).WillRepeatedly(ReturnPointee(&expected_final_state));
  EXPECT_CALL(filter, importance_weight(::testing::_)).WillRepeatedly(Return(weight_reduction_factor));
  EXPECT_CALL(filter, do_resampling_vote()).WillRepeatedly(Return(true));

  for (auto iteration = 0; iteration < 4; ++iteration) {
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_initial_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(expected_initial_weight));

    // apply motion on all particles, particles will have updated states, but unchanged weight
    filter.sample();
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_final_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(expected_initial_weight));

    // updating particle weights, particles will have updated weights, but unchanged state
    filter.reweight();
    ASSERT_THAT(filter.states() | ranges::to<std::vector>, Each(expected_final_state));
    ASSERT_THAT(filter.weights() | ranges::to<std::vector>, Each(expected_final_weight));

    // resample, resampling will reset weights
    filter.resample();

    expected_initial_state = expected_final_state;
    expected_final_state += motion_increment;
  }
}

}  // namespace
