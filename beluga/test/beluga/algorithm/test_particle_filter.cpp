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
#include <beluga/views.hpp>
#include <ciabatta/ciabatta.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>
#include <range/v3/view/transform.hpp>

namespace {

using testing::Each;
using testing::Return;

template <class Mixin>
class MockMixin : public Mixin {
 public:
  MOCK_METHOD(double, apply_motion, (double state), (const));
  MOCK_METHOD(double, importance_weight, (double state), (const));
  MOCK_METHOD(bool, do_resampling_vote, ());

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
    return ranges::views::generate([]() { return 0.0; });
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

TEST(BootstrapParticleFilter, Update) {
  auto filter = ParticleFilter{};
  EXPECT_CALL(filter, apply_motion(0.)).WillRepeatedly(Return(2.));
  EXPECT_CALL(filter, importance_weight(2.)).WillRepeatedly(Return(3.));
  EXPECT_THAT(filter.states() | ranges::to<std::vector>, Each(0.));
  filter.sample();
  EXPECT_THAT(filter.states() | ranges::to<std::vector>, Each(2.));
  EXPECT_THAT(filter.weights() | ranges::to<std::vector>, Each(1.));
  filter.importance_sample();
  EXPECT_THAT(filter.weights() | ranges::to<std::vector>, Each(3.));
}

}  // namespace
