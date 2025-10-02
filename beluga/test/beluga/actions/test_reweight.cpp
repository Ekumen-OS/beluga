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
#include <gtest/gtest.h>

#include <execution>
#include <functional>
#include <tuple>
#include <vector>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/intersperse.hpp>

#include "beluga/actions/assign.hpp"
#include "beluga/actions/reweight.hpp"
#include "beluga/primitives.hpp"
#include "beluga/views/particles.hpp"

namespace {

TEST(ReweightAction, DefaultExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(2.0))};
  input |= beluga::actions::reweight([](int value) { return value; });
  ASSERT_EQ(input.front(), std::make_tuple(5, 10.0));
}

TEST(ReweightAction, SequencedExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(2.0))};
  input |= beluga::actions::reweight(std::execution::seq, [](int value) { return value; });
  ASSERT_EQ(input.front(), std::make_tuple(5, 10.0));
}

TEST(ReweightAction, ParallelExecutionPolicy) {
  auto input = std::vector{std::make_tuple(5, beluga::Weight(2.0))};
  input |= beluga::actions::reweight(std::execution::par, [](int value) { return value; });
  ASSERT_EQ(input.front(), std::make_tuple(5, 10.0));
}

TEST(ReweightAction, Composition) {
  auto input = std::vector{std::make_tuple(4, beluga::Weight(0.5)), std::make_tuple(4, beluga::Weight(1.0))};
  input |= beluga::actions::reweight([](int value) { return value; }) |           //
           ranges::views::intersperse(std::make_tuple(5, beluga::Weight(1.0))) |  //
           beluga::actions::assign;
  auto weights = input | beluga::views::weights | ranges::to<std::vector>;
  ASSERT_THAT(weights, testing::ElementsAre(2, 1, 4));
}

TEST(ReweightAction, StatefulModel) {
  auto input = std::vector{std::make_tuple(4, beluga::Weight(0.5)), std::make_tuple(4, beluga::Weight(1.0))};
  auto model = [value = 0](int) mutable { return value++; };
  input |= ranges::views::intersperse(std::make_tuple(5, beluga::Weight(1.0))) |  //
           beluga::actions::assign |                                              //
           beluga::actions::reweight(std::ref(model));
  auto weights = input | beluga::views::weights | ranges::to<std::vector>;
  ASSERT_THAT(weights, testing::ElementsAre(0, 1, 2));
  ASSERT_EQ(model(0), 3);  // the model was passed by reference
}

// A simple particle struct for testing the new overload (tracking the likelihood)
// Needed because std::tuple doesn't support pointer-to-member access.
struct TestParticle {
  int state;
  beluga::Weight weight;
  double likelihood_score = 0.0;

  // Add equality operator for easy comparison in tests
  bool operator==(const TestParticle& other) const {
    return state == other.state && weight == other.weight && likelihood_score == other.likelihood_score;
  }
};

// Default test case to verify core functionality
TEST(ReweightAction, StoreLikelihoodDefault) {
  auto input = std::vector<TestParticle>{{5, beluga::Weight(2.0), 0.0}};
  input |=
      beluga::actions::reweight([](int value) { return static_cast<double>(value); }, &TestParticle::likelihood_score);

  ASSERT_DOUBLE_EQ(input.front().weight, 10.0);
  ASSERT_DOUBLE_EQ(input.front().likelihood_score, 5.0);
}

// Composition test to ensure the action is pipeable
TEST(ReweightAction, StoreLikelihoodComposition) {
  auto particles = std::vector<TestParticle>{
      {2, beluga::Weight(1.0), 0.0},  // Likelihood will be 2.0 (keep)
      {0, beluga::Weight(1.0), 0.0},  // Likelihood will be 0.0 (remove)
      {4, beluga::Weight(1.0), 0.0}   // Likelihood will be 4.0 (keep)
  };
  auto model = [](int state) { return static_cast<double>(state); };

  // A single compound action that reweights, filters, and assigns.
  particles |= beluga::actions::reweight(model, &TestParticle::likelihood_score) |
               ranges::views::filter([](const auto& p) { return p.likelihood_score >= 1.0; }) | beluga::actions::assign;

  // Verify that the filtering worked.
  ASSERT_EQ(particles.size(), 2);
  ASSERT_EQ(particles[0].state, 2);
  ASSERT_EQ(particles[0].weight, 2.0);  // 1.0 * 2.0
  ASSERT_EQ(particles[0].likelihood_score, 2.0);
  ASSERT_EQ(particles[1].state, 4);
  ASSERT_EQ(particles[1].weight, 4.0);  // 1.0 * 4.0
  ASSERT_EQ(particles[1].likelihood_score, 4.0);
}

// Genericity test with a different particle struct, to prove it can work with different likelihood types and member
// names
struct AnotherParticle {
  int state;
  beluga::Weight weight;
  float score = 0.0F;
};

TEST(ReweightAction, StoreLikelihoodInDifferentMember) {
  auto input = std::vector<AnotherParticle>{{10, beluga::Weight(0.5), 0.0F}};
  auto model = [](int value) { return static_cast<float>(value); };

  input |= beluga::actions::reweight(model, &AnotherParticle::score);

  ASSERT_EQ(input.front().weight, 5.0);  // 0.5 * 10.0
  ASSERT_FLOAT_EQ(input.front().score, 10.0F);
}

}  // namespace
