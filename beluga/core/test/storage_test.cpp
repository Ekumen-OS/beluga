#include <iostream>

#include <gmock/gmock.h>

#include <beluga/storage.h>

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
class StoragePolicyTest : public testing::Test {};

using Implementations =
    testing::Types<beluga::core::storage::ArrayOfStructures<State>, beluga::core::storage::StructureOfArrays<State>>;
TYPED_TEST_SUITE(StoragePolicyTest, Implementations);

TYPED_TEST(StoragePolicyTest, Resize) {
  using StoragePolicy = TypeParam;
  using Container = typename StoragePolicy::container_type;
  Container container;
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  for (auto&& state : StoragePolicy::state_view(container)) {
    EXPECT_THAT(state, StateEq(State{}));
  }
}

TYPED_TEST(StoragePolicyTest, Clear) {
  using StoragePolicy = TypeParam;
  using Container = typename StoragePolicy::container_type;
  Container container;
  container.resize(15);
  EXPECT_EQ(container.size(), 15);
  container.clear();
  EXPECT_EQ(container.size(), 0);
}

TYPED_TEST(StoragePolicyTest, PushBack) {
  using StoragePolicy = TypeParam;
  using Container = typename StoragePolicy::container_type;
  Container container;
  constexpr auto kTestState = State{1, 2};
  EXPECT_EQ(container.size(), 0);
  container.push_back({kTestState, 0, 0});
  EXPECT_EQ(container.size(), 1);
  EXPECT_THAT(*std::begin(StoragePolicy::state_view(container)), StateEq(kTestState));
}

TYPED_TEST(StoragePolicyTest, StateView) {
  using StoragePolicy = TypeParam;
  using Container = typename StoragePolicy::container_type;
  Container container;
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr auto kTestState = State{1, 2};
  for (auto&& state : StoragePolicy::state_view(container)) {
    state = kTestState;
  }
  for (auto&& state : StoragePolicy::state_view(container)) {
    EXPECT_THAT(state, StateEq(kTestState));
  }
}

TYPED_TEST(StoragePolicyTest, WeightView) {
  using StoragePolicy = TypeParam;
  using Container = typename StoragePolicy::container_type;
  Container container;
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr double kTestWeight = 1.25;
  for (auto&& weight : StoragePolicy::weight_view(container)) {
    weight = kTestWeight;
  }
  for (auto&& weight : StoragePolicy::weight_view(container)) {
    EXPECT_EQ(weight, kTestWeight);
  }
}

TYPED_TEST(StoragePolicyTest, ClusterView) {
  using StoragePolicy = TypeParam;
  using Container = typename StoragePolicy::container_type;
  Container container;
  container.resize(3);
  EXPECT_EQ(container.size(), 3);
  constexpr std::size_t kTestCluster = 75;
  for (auto&& cluster_id : StoragePolicy::cluster_view(container)) {
    cluster_id = kTestCluster;
  }
  for (auto&& cluster_id : StoragePolicy::cluster_view(container)) {
    EXPECT_EQ(cluster_id, kTestCluster);
  }
}

}  // namespace
