// Copyright 2024 Ekumen, Inc.
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

#include "beluga/type_traits/particle_traits.hpp"
#include "beluga/type_traits/tuple_traits.hpp"

#include <range/v3/utility/common_tuple.hpp>

namespace {

static_assert(!beluga::is_particle_v<int>);
static_assert(!beluga::is_particle_v<struct Object>);
static_assert(!beluga::is_particle_v<std::tuple<int, int>>);
static_assert(!beluga::is_particle_v<std::tuple<beluga::Weight>>);
static_assert(!beluga::is_particle_v<std::tuple<beluga::Weight, beluga::Weight>>);

namespace user {

struct SimplestPossibleParticle {
  int state;
  double weight;
};

struct ParticleWithMemberExtensions {
  int state_;
  double weight_;

  int& state() { return state_; }
  [[nodiscard]] int state() const { return state_; }
  double& weight() { return weight_; }
  [[nodiscard]] double weight() const { return weight_; }
};

struct ParticleWithNonMemberExtensions {
  int s;
  double w;
};

/*
 * The following functions are marked [[maybe_unused]] since the compiler does not
 * seem to detect that ADL is discovering them. But they are actually used.
 */

[[maybe_unused]] int& state(ParticleWithNonMemberExtensions& p) {
  return p.s;
}
[[maybe_unused]] int state(const ParticleWithNonMemberExtensions& p) {
  return p.s;
}
[[maybe_unused]] double& weight(ParticleWithNonMemberExtensions& p) {
  return p.w;
}
[[maybe_unused]] double weight(const ParticleWithNonMemberExtensions& p) {
  return p.w;
}

}  // namespace user

template <class T>
class ParticleTraitsTest : public testing::Test {};

using ParticleTraitsTestCases = testing::Types<
    std::tuple<int, beluga::Weight>,
    std::pair<int, beluga::Weight>,
    ranges::common_tuple<int, beluga::Weight>,
    ranges::common_pair<int, beluga::Weight>,
    user::SimplestPossibleParticle,
    user::ParticleWithMemberExtensions,
    user::ParticleWithNonMemberExtensions>;

TYPED_TEST_SUITE(ParticleTraitsTest, ParticleTraitsTestCases, );

TYPED_TEST(ParticleTraitsTest, MakeFromState) {
  // Also check that each parameter type is a valid particle.
  static_assert(beluga::is_particle_v<TypeParam>);
  auto particle = beluga::make_from_state<TypeParam>(5);
  ASSERT_EQ(beluga::state(particle), 5);
  ASSERT_EQ(beluga::weight(particle), 1.0);
}

}  // namespace
