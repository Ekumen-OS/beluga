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

template <typename T>
class ContainerTest : public testing::Test {};

using Implementations = testing::
    Types<beluga::TupleVector<State, double, std::size_t>, std::vector<std::tuple<State, double, std::size_t>>>;
TYPED_TEST_SUITE(ContainerTest, Implementations);

TYPED_TEST(ContainerTest, Resize) {
  auto container = TypeParam{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  auto&& states = beluga::views::all(container) | beluga::views::elements<0>;
  for (auto&& state : states) {
    ASSERT_THAT(state, StateEq(State{}));
  }
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
  constexpr auto kTestState = State{1, 2};
  EXPECT_EQ(container.size(), 0);
  container.push_back({kTestState, 0, 0});
  EXPECT_EQ(container.size(), 1);
  auto&& states = beluga::views::all(container) | beluga::views::elements<0>;
  EXPECT_THAT(*std::begin(states), StateEq(kTestState));
}

TYPED_TEST(ContainerTest, StateView) {
  auto container = TypeParam{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr auto kTestState = State{1, 2};
  for (auto&& state : beluga::views::all(container) | beluga::views::elements<0>) {
    state = kTestState;
  }
  for (auto&& state : beluga::views::all(container) | beluga::views::elements<0>) {
    ASSERT_THAT(state, StateEq(kTestState));
  }
}

TYPED_TEST(ContainerTest, WeightView) {
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

TYPED_TEST(ContainerTest, ClusterView) {
  auto container = TypeParam{};
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr std::size_t kTestCluster = 75;
  for (auto&& cluster_id : beluga::views::all(container) | beluga::views::elements<2>) {
    cluster_id = kTestCluster;
  }
  for (auto&& cluster_id : beluga::views::all(container) | beluga::views::elements<2>) {
    ASSERT_EQ(cluster_id, kTestCluster);
  }
}

}  // namespace
