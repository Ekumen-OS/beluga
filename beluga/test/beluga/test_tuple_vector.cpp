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

#include <beluga/tuple_vector.hpp>
#include <beluga/type_traits.hpp>

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
  using TemplateParam = T<Types...>;
};

template <class T>
class ContainerTest : public testing::Test {
 public:
  template <class... Types>
  using TemplateParam = typename T::template TemplateParam<Types...>;
};

using Implementations = testing::Types<TemplateWrapper<beluga::VectorOfTuples>, TemplateWrapper<beluga::TupleOfVectors>>;
TYPED_TEST_SUITE(ContainerTest, Implementations, );

TYPED_TEST(ContainerTest, Size) {
  auto container = typename TestFixture::template TemplateParam<State, double, std::size_t>{100};
  ASSERT_EQ(container.size(), 100);
}

TYPED_TEST(ContainerTest, Resize) {
  auto container = typename TestFixture::template TemplateParam<State, double, std::size_t>{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  int reps = 0;
  for (auto&& [state, weight, cluster] : beluga::views::all(container)) {
    ASSERT_THAT(state, StateEq(State{}));
    ASSERT_EQ(weight, double{});
    ASSERT_EQ(cluster, std::size_t{});
    ++reps;
  }
  ASSERT_EQ(reps, 3);
}

TYPED_TEST(ContainerTest, Clear) {
  auto container = typename TestFixture::template TemplateParam<State, double, std::size_t>{};
  container.resize(15);
  EXPECT_EQ(container.size(), 15);
  container.clear();
  EXPECT_EQ(container.size(), 0);
}

TYPED_TEST(ContainerTest, PushBack) {
  auto container = typename TestFixture::template TemplateParam<State, double, std::size_t>{};
  EXPECT_EQ(container.size(), 0);
  container.push_back({{1, 2}, 3, 4});
  EXPECT_EQ(container.size(), 1);
  auto&& [state, weight, cluster] = beluga::views::all(container).front();
  EXPECT_THAT(state, StateEq({1, 2}));
  EXPECT_EQ(weight, 3);
  EXPECT_EQ(cluster, 4);
}

TYPED_TEST(ContainerTest, GetterSetter) {
  auto container = typename TestFixture::template TemplateParam<State, double, std::size_t>{};
  auto&& view = beluga::views::all(container);
  container.push_back({{1, 2}, 3, 4});
  beluga::weight(view.front()) = 5;
  ASSERT_EQ(beluga::weight(view.front()), 5);
}

TYPED_TEST(ContainerTest, View) {
  auto container = typename TestFixture::template TemplateParam<State, double, std::size_t>{};
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
  auto container = typename TestFixture::template TemplateParam<State, double, std::size_t>{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr std::size_t kTestCluster = 75;
  for (auto&& [state, weight, cluster] : beluga::views::all(container)) {
    cluster = kTestCluster;
  }
  for (auto&& [state, weight, cluster] : beluga::views::all(container)) {
    ASSERT_EQ(cluster, kTestCluster);
  }
}

TYPED_TEST(ContainerTest, SingleElementTuple) {
  auto container = typename TestFixture::template TemplateParam<int>{};
  container.push_back({4});
  container.push_back({4});
  container.push_back({4});
  EXPECT_EQ(container.size(), 3);
  for (auto&& [value] : beluga::views::all(container)) {
    ASSERT_EQ(value, 4);
  }
}

TYPED_TEST(ContainerTest, DoubleElementTuple) {
  auto container = typename TestFixture::template TemplateParam<int, float>{};
  container.push_back({2, 0.5});
  container.push_back({2, 0.5});
  container.push_back({2, 0.5});
  EXPECT_EQ(container.size(), 3);
  for (auto&& [integer, real] : beluga::views::all(container)) {
    ASSERT_EQ(integer, 2);
    ASSERT_EQ(real, 0.5);
  }
}

}  // namespace
