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

#include <gtest/gtest.h>

#include <beluga/primitives.hpp>
#include <range/v3/utility/common_tuple.hpp>

namespace {

namespace user {

struct SimplestPossibleParticle {
  float state;
  double weight;
  int cluster;
};

struct ParticleWithMemberExtensions {
  float state_;
  double weight_;
  int cluster_;

  float& state() { return state_; }
  [[nodiscard]] float state() const { return state_; }
  double& weight() { return weight_; }
  [[nodiscard]] double weight() const { return weight_; }
  int& cluster() { return cluster_; }
  [[nodiscard]] int cluster() const { return cluster_; }
};

struct ParticleWithNonMemberExtensions {
  float s;
  double w;
  int c;
};

/*
 * The following functions are marked [[maybe_unused]] since the compiler does not
 * seem to detect that ADL is discovering them. But they are actually used.
 */

[[maybe_unused]] float& state(ParticleWithNonMemberExtensions& p) {
  return p.s;
}
[[maybe_unused]] float state(const ParticleWithNonMemberExtensions& p) {
  return p.s;
}
[[maybe_unused]] double& weight(ParticleWithNonMemberExtensions& p) {
  return p.w;
}
[[maybe_unused]] double weight(const ParticleWithNonMemberExtensions& p) {
  return p.w;
}
[[maybe_unused]] int& cluster(ParticleWithNonMemberExtensions& p) {
  return p.c;
}
[[maybe_unused]] int cluster(const ParticleWithNonMemberExtensions& p) {
  return p.c;
}

}  // namespace user

template <class T>
class PrimitivesTest : public testing::Test {};

using PrimitivesTestCases = testing::Types<
    std::tuple<int, beluga::Weight, beluga::Cluster>,
    ranges::common_tuple<int, beluga::Weight, beluga::Cluster>,
    user::SimplestPossibleParticle,
    user::ParticleWithMemberExtensions,
    user::ParticleWithNonMemberExtensions>;

TYPED_TEST_SUITE(PrimitivesTest, PrimitivesTestCases, );

TYPED_TEST(PrimitivesTest, Assignment) {
  auto particle = TypeParam{};
  beluga::state(particle) = 1;
  beluga::weight(particle) = 2;
  beluga::cluster(particle) = 3;
  ASSERT_EQ(beluga::state(particle), 1);
  ASSERT_EQ(beluga::weight(particle), 2);
  ASSERT_EQ(beluga::cluster(particle), 3);
}

TYPED_TEST(PrimitivesTest, Const) {
  const auto particle = TypeParam{4, 5, 6};
  ASSERT_EQ(beluga::state(particle), 4);
  ASSERT_EQ(beluga::weight(particle), 5);
  ASSERT_EQ(beluga::cluster(particle), 6);
}

}  // namespace
