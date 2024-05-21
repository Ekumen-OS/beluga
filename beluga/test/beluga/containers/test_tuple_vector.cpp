// Copyright 2022-2024 Ekumen, Inc.
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
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <ostream>
#include <tuple>
#include <type_traits>
#include <utility>

#include <range/v3/algorithm/equal.hpp>
#include <range/v3/iterator/access.hpp>
#include <range/v3/range/access.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/range/traits.hpp>
#include <range/v3/utility/common_tuple.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/const.hpp>
#include <range/v3/view/filter.hpp>

#include "beluga/containers/tuple_vector.hpp"
#include "beluga/primitives.hpp"
#include "beluga/views/particles.hpp"

namespace {

struct State {
  double x = 0.;
  double y = 0.;
};

inline void PrintTo(const State& state, std::ostream* os) {
  *os << "A state (x: " << state.x << ", y: " << state.y << ")";
}

inline auto StateEq(const State& expected) {
  using namespace testing;  // NOLINT
  return AllOf(Field("x", &State::x, DoubleEq(expected.x)), Field("y", &State::y, DoubleEq(expected.y)));
}

template <template <class...> class T>
struct TemplateWrapper {
  template <class... Types>
  using type = T<std::tuple<Types...>>;
};

template <class T>
class ContainerTest : public testing::Test {
 public:
  template <class... Types>
  using TemplateParam = typename T::template type<Types...>;
};

using Implementations = testing::Types<TemplateWrapper<beluga::Vector>, TemplateWrapper<beluga::TupleVector>>;
TYPED_TEST_SUITE(ContainerTest, Implementations, );

TYPED_TEST(ContainerTest, Size) {
  auto container = typename TestFixture::template TemplateParam<State, beluga::Weight, beluga::Cluster>{100};
  ASSERT_EQ(container.size(), 100);
}

TYPED_TEST(ContainerTest, Resize) {
  auto container = typename TestFixture::template TemplateParam<State, beluga::Weight, beluga::Cluster>{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  int reps = 0;
  for (auto&& [state, weight, cluster] : container) {
    ASSERT_THAT(state, StateEq(State{}));
    ASSERT_EQ(weight, double{});
    ASSERT_EQ(cluster, beluga::Cluster{});
    ++reps;
  }
  ASSERT_EQ(reps, 3);
}

TYPED_TEST(ContainerTest, Clear) {
  auto container = typename TestFixture::template TemplateParam<State, beluga::Weight, beluga::Cluster>{};
  container.resize(15);
  EXPECT_EQ(container.size(), 15);
  container.clear();
  EXPECT_EQ(container.size(), 0);
}

TYPED_TEST(ContainerTest, PushBack) {
  auto container = typename TestFixture::template TemplateParam<State, beluga::Weight, beluga::Cluster>{};
  EXPECT_EQ(container.size(), 0);
  container.push_back({{1, 2}, 3, 4});
  EXPECT_EQ(container.size(), 1);
  auto&& [state, weight, cluster] = ranges::views::all(container).front();
  EXPECT_THAT(state, StateEq({1, 2}));
  EXPECT_EQ(weight, 3);
  EXPECT_EQ(cluster, 4);
}

TYPED_TEST(ContainerTest, GetterSetter) {
  auto container = typename TestFixture::template TemplateParam<State, beluga::Weight, beluga::Cluster>{};
  auto&& view = container | ranges::views::all;
  container.push_back({{1, 2}, 3, 4});
  beluga::weight(view.front()) = 5;
  ASSERT_EQ(beluga::weight(view.front()), 5);
}

TYPED_TEST(ContainerTest, View) {
  auto container = typename TestFixture::template TemplateParam<State, beluga::Weight, beluga::Cluster>{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr double kTestWeight = 1.25;
  for (auto&& weight : beluga::views::weights(container)) {
    weight = kTestWeight;
  }
  for (auto&& weight : beluga::views::weights(container)) {
    ASSERT_EQ(weight, kTestWeight);
  }
}

TYPED_TEST(ContainerTest, StructuredBinding) {
  auto container = typename TestFixture::template TemplateParam<State, beluga::Weight, beluga::Cluster>{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr std::size_t kTestCluster = 75;
  for (auto&& [state, weight, cluster] : container) {
    cluster = kTestCluster;
  }
  for (auto&& [state, weight, cluster] : container) {
    ASSERT_EQ(cluster, kTestCluster);
  }
}

TYPED_TEST(ContainerTest, SingleElementTuple) {
  auto container = typename TestFixture::template TemplateParam<int>{};
  container.push_back({4});
  container.push_back({4});
  container.push_back({4});
  EXPECT_EQ(container.size(), 3);
  for (auto&& [value] : container) {
    ASSERT_EQ(value, 4);
  }
}

TYPED_TEST(ContainerTest, DoubleElementTuple) {
  auto container = typename TestFixture::template TemplateParam<int, float>{};
  container.push_back({2, 0.5});
  container.push_back({2, 0.5});
  container.push_back({2, 0.5});
  EXPECT_EQ(container.size(), 3);
  for (auto&& [integer, real] : container) {
    ASSERT_EQ(integer, 2);
    ASSERT_EQ(real, 0.5);
  }
}

TEST(TupleVectorTest, ConceptChecks) {
  auto container = beluga::TupleVector<std::tuple<float>>{std::make_tuple(1)};

  static_assert(ranges::sized_range<decltype(container)>);
  static_assert(ranges::random_access_range<decltype(container)>);
  static_assert(ranges::common_range<decltype(container)>);
  static_assert(ranges::bidirectional_range<decltype(container)>);

  static_assert(!ranges::viewable_range<decltype(container)>);
  static_assert(!ranges::contiguous_range<decltype(container)>);
}

TEST(TupleVectorTest, TraitConsistency) {
  using Container = beluga::TupleVector<std::tuple<float, int>>;
  using ConstContainer = decltype(std::declval<const Container&>());
  using ConstView = decltype(std::declval<const Container&>() | ranges::views::const_);
  using Iterator = decltype(ranges::begin(std::declval<Container&>()));

  // Expected types
  static_assert(std::is_same_v<
                ranges::range_value_t<Container>,  //
                std::tuple<float, int>>);
  static_assert(std::is_same_v<
                ranges::range_reference_t<Container>,  //
                ranges::common_tuple<float&, int&>>);
  static_assert(std::is_same_v<
                ranges::range_rvalue_reference_t<Container>,  //
                ranges::common_tuple<float&&, int&&>>);
  static_assert(std::is_same_v<
                ranges::range_value_t<ConstContainer>,  //
                std::tuple<float, int>>);
  static_assert(std::is_same_v<
                ranges::range_reference_t<ConstContainer>,  //
                ranges::common_tuple<const float&, const int&>>);
  static_assert(std::is_same_v<
                ranges::range_rvalue_reference_t<ConstContainer>,  //
                ranges::common_tuple<const float&&, const int&&>>);

  // Expected value type of a const view would be the same as the value type of the
  // adapted container (std::tuple<float, int>)... This is not the case. :(
  static_assert(std::is_same_v<
                ranges::range_value_t<ConstView>,  //
                ranges::common_tuple<const float&, const int&>>);

  // Consistency
  static_assert(std::is_same_v<
                ranges::iter_value_t<Iterator>,  //
                typename Container::value_type>);
  static_assert(std::is_same_v<
                ranges::iter_reference_t<Iterator>,  //
                typename Container::reference_type>);
}

TEST(TupleVectorTest, ConstCorrectness) {
  auto container = beluga::TupleVector<std::tuple<float>>{std::make_tuple(1)};
  static_assert(std::is_same_v<decltype(*ranges::begin(container)), ranges::common_tuple<float&>>);
  static_assert(std::is_same_v<decltype(*ranges::begin(std::as_const(container))), ranges::common_tuple<const float&>>);
}

TEST(TupleVectorTest, ConversionFromSizedRange) {
  auto input = std::array{std::make_tuple(1, 1.0), std::make_tuple(2, 2.0), std::make_tuple(3, 3.0)};
  auto output = input | ranges::to<beluga::TupleVector>;
  ASSERT_TRUE(ranges::equal(input, output));
}

TEST(TupleVectorTest, ConversionFromNonSizedRange) {
  auto input = std::array{std::make_tuple(1, 1.0), std::make_tuple(2, 2.0), std::make_tuple(3, 3.0)};
  auto output = input | ranges::views::filter([](auto) { return true; }) | ranges::to<beluga::TupleVector>;
  ASSERT_TRUE(ranges::equal(input, output));
}

TEST(TupleVectorTest, AssignFromSizedRange) {
  auto input = std::array{std::make_tuple(1, 1.0), std::make_tuple(2, 2.0), std::make_tuple(3, 3.0)};
  auto output = beluga::TupleVector<std::tuple<int, double>>{};
  output.reserve(2);
  output.assign_range(input);
  ASSERT_TRUE(ranges::equal(input, output));
}

TEST(TupleVectorTest, AssignFromNonSizedRange) {
  auto input = std::array{std::make_tuple(1, 1.0), std::make_tuple(2, 2.0), std::make_tuple(3, 3.0)};
  auto output = beluga::TupleVector<std::tuple<int, double>>{};
  output.reserve(2);
  output.assign_range(input | ranges::views::filter([](auto) { return true; }));
  ASSERT_TRUE(ranges::equal(input, output));
}

TEST(TupleVectorTest, AssignFromSizedRangeReserved) {
  auto input = std::array{std::make_tuple(1, 1.0), std::make_tuple(2, 2.0), std::make_tuple(3, 3.0)};
  auto output = beluga::TupleVector<std::tuple<int, double>>{};
  output.reserve(4);
  output.assign_range(input);
  ASSERT_TRUE(ranges::equal(input, output));
}

TEST(TupleVectorTest, AssignFromNonSizedRangeReserved) {
  auto input = std::array{std::make_tuple(1, 1.0), std::make_tuple(2, 2.0), std::make_tuple(3, 3.0)};
  auto output = beluga::TupleVector<std::tuple<int, double>>{};
  output.reserve(4);
  output.assign_range(input | ranges::views::filter([](auto) { return true; }));
  ASSERT_TRUE(ranges::equal(input, output));
}

}  // namespace
