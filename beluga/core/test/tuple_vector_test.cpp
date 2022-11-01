#include <iostream>

#include <gmock/gmock.h>

#include <beluga/tuple_vector.h>
#include <beluga/views.h>

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

template <class T>
class ContainerTest : public testing::Test {};

using Particle = std::tuple<State, double, std::size_t>;
using StructureOfArrays = beluga::TupleVector<Particle>;
using ArrayOfStructures = std::vector<Particle>;

using Implementations = testing::Types<StructureOfArrays, ArrayOfStructures>;
TYPED_TEST_SUITE(ContainerTest, Implementations);

TYPED_TEST(ContainerTest, Resize) {
  auto container = TypeParam{};
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
  auto container = TypeParam{};
  container.resize(15);
  EXPECT_EQ(container.size(), 15);
  container.clear();
  EXPECT_EQ(container.size(), 0);
}

TYPED_TEST(ContainerTest, PushBack) {
  auto container = TypeParam{};
  EXPECT_EQ(container.size(), 0);
  container.push_back({{1, 2}, 3, 4});
  EXPECT_EQ(container.size(), 1);
  auto&& [state, weight, cluster] = beluga::views::all(container).front();
  EXPECT_THAT(state, StateEq({1, 2}));
  EXPECT_EQ(weight, 3);
  EXPECT_EQ(cluster, 4);
}

TYPED_TEST(ContainerTest, ElementsView) {
  auto container = TypeParam{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr double kTestWeight = 1.25;
  for (auto&& weight : beluga::views::all(container) | beluga::views::elements<1>) {
    weight = kTestWeight;
  }
  for (auto&& weight : beluga::views::all(container) | beluga::views::elements<1>) {
    ASSERT_EQ(weight, kTestWeight);
  }
}

TYPED_TEST(ContainerTest, StructuredBinding) {
  auto container = TypeParam{};
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

TEST(TupleVector, SingleElementTuple) {
  auto container = beluga::TupleVector<std::tuple<int>>{};
  container.push_back({4});
  container.push_back({4});
  container.push_back({4});
  EXPECT_EQ(container.size(), 3);
  for (auto&& [value] : beluga::views::all(container)) {
    ASSERT_EQ(value, 4);
  }
}

TEST(TupleVector, DoubleElementTuple) {
  auto container = beluga::TupleVector<std::tuple<int, float>>{};
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
