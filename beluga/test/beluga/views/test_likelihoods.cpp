// Copyright 2025 Ekumen, Inc.
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

#include <vector>

#include <range/v3/range/concepts.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>

#include "beluga/primitives.hpp"
#include "beluga/views/likelihoods.hpp"

namespace {

// Use a simple tuple-based particle for testing, consistent with other tests.
using TestParticle = std::tuple<int, beluga::Weight>;

TEST(LikelihoodsView, CorrectlyTransformsStates) {
  // Create a set of particles and a simple sensor model.
  const auto particles = std::vector<TestParticle>{{10, 0.5}, {20, 0.4}, {30, 0.1}};
  // The model simply doubles the state value.
  auto model = [](int state) { return static_cast<double>(state * 2); };

  // Apply the likelihoods view.
  auto likelihoods_view = particles | beluga::views::likelihoods(model);

  // The view should contain the results of applying the model to each particle's state.
  const auto expected_likelihoods = std::vector<double>{20.0, 40.0, 60.0};
  ASSERT_THAT(likelihoods_view | ranges::to<std::vector>, testing::ElementsAreArray(expected_likelihoods));
}

TEST(LikelihoodsView, IsLazy) {
  // Create particles and a model along with a call counter.
  const auto particles = std::vector<TestParticle>{{1, 0.5}, {2, 0.5}};
  int call_count = 0;
  auto model = [&](int state) {
    call_count++;
    return static_cast<double>(state);
  };

  // Create the view.
  auto likelihoods_view = particles | beluga::views::likelihoods(model);

  // The model should not have been called yet.
  ASSERT_EQ(call_count, 0);

  // Iterate over the view.
  auto result = likelihoods_view | ranges::to<std::vector>;

  // The model should now have been called for each particle.
  ASSERT_EQ(call_count, 2);
  ASSERT_EQ(result.size(), 2);
}

TEST(LikelihoodsView, ConceptChecks) {
  // Create a vector of particles (std::vector is well known and fulfills several range concepts).
  const auto particles = std::vector<TestParticle>{};
  auto model = [](int state) { return static_cast<double>(state); };
  auto output = particles | beluga::views::likelihoods(model);

  // The likelihoods view, being a transform, it's structure-preserving and should preserve the properties
  // of the input range.
  static_assert(ranges::viewable_range<decltype(output)>);
  static_assert(ranges::sized_range<decltype(output)>);
  static_assert(ranges::forward_range<decltype(output)>);
  static_assert(ranges::bidirectional_range<decltype(output)>);
  static_assert(ranges::random_access_range<decltype(output)>);

  // The size of the output view should be the same as the input.
  ASSERT_EQ(ranges::size(particles), ranges::size(output));
}

TEST(LikelihoodsView, ComposesWithOtherViews) {
  const auto particles = std::vector<TestParticle>{{5, 0.5}, {15, 0.25}, {25, 0.25}};
  auto model = [](int state) { return static_cast<double>(state); };

  // Create the likelihoods view and pipe it into another view (a filter for the likelihoods).
  auto filtered_likelihoods = particles | beluga::views::likelihoods(model) |
                              ranges::views::filter([](double likelihood) { return likelihood > 10.0; });

  // The final output should be correct after composition.
  const auto expected_likelihoods = std::vector<double>{15.0, 25.0};
  ASSERT_THAT(filtered_likelihoods | ranges::to<std::vector>, testing::ElementsAreArray(expected_likelihoods));
}

}  // namespace
