// Copyright 2023-2024 Ekumen, Inc.
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

#include <beluga/mixin/storage.hpp>
#include <ciabatta/ciabatta.hpp>

#include <range/v3/algorithm/equal.hpp>
#include <range/v3/view/reverse.hpp>

namespace {

using testing::Return;

template <class Mixin>
class MockMixin : public Mixin {
 public:
  MOCK_METHOD(std::size_t, max_samples, (), (const));
};

using SOA_WITH_PAIR = ciabatta::mixin<
    ciabatta::curry<beluga::StructureOfArrays, int, beluga::Weight>::mixin,
    MockMixin,
    ciabatta::provides<beluga::StorageInterface<int, beluga::Weight>>::mixin>;

using AOS_WITH_PAIR = ciabatta::mixin<
    ciabatta::curry<beluga::ArrayOfStructures, int, beluga::Weight>::mixin,
    MockMixin,
    ciabatta::provides<beluga::StorageInterface<int, beluga::Weight>>::mixin>;

using SOA_WITH_TUPLE = ciabatta::mixin<
    ciabatta::curry<beluga::StructureOfArrays, int, beluga::Weight, beluga::Cluster>::mixin,
    MockMixin,
    ciabatta::provides<beluga::StorageInterface<int, beluga::Weight>>::mixin>;

using AOS_WITH_TUPLE = ciabatta::mixin<
    ciabatta::curry<beluga::ArrayOfStructures, int, beluga::Weight, beluga::Cluster>::mixin,
    MockMixin,
    ciabatta::provides<beluga::StorageInterface<int, beluga::Weight>>::mixin>;

template <class T>
class StoragePolicyTest : public testing::Test {};

using Implementations = testing::Types<SOA_WITH_PAIR, AOS_WITH_PAIR, SOA_WITH_TUPLE, AOS_WITH_TUPLE>;
TYPED_TEST_SUITE(StoragePolicyTest, Implementations, );

TYPED_TEST(StoragePolicyTest, InitializeWithLessParticlesThanExpected) {
  auto states = std::vector<int>{1, 2, 3};
  auto mixin = TypeParam{};
  EXPECT_CALL(mixin, max_samples()).WillOnce(Return(5));
  mixin.initialize_states(states);
  ASSERT_EQ(mixin.particle_count(), 3);
}

TYPED_TEST(StoragePolicyTest, InitializeWithMoreParticlesThanExpected) {
  auto states = std::vector<int>{1, 2, 3, 4};
  auto mixin = TypeParam{};
  EXPECT_CALL(mixin, max_samples()).WillOnce(Return(2));
  mixin.initialize_states(states);
  ASSERT_EQ(mixin.particle_count(), 2);
}

TYPED_TEST(StoragePolicyTest, ResampleParticles) {
  auto states = std::vector<int>{1, 2, 3, 4, 5};
  auto mixin = TypeParam{};
  EXPECT_CALL(mixin, max_samples()).WillOnce(Return(5)).WillOnce(Return(5));
  mixin.initialize_states(states);
  ASSERT_EQ(mixin.particle_count(), 5);
  mixin.initialize_particles(mixin.particles() | ranges::views::reverse);
  ASSERT_TRUE(ranges::equal(mixin.states(), states | ranges::views::reverse));
}

}  // namespace
