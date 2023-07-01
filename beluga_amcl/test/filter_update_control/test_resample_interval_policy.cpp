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

#include <ciabatta/ciabatta.hpp>

#include <beluga_amcl/filter_update_control/resample_interval_policy.hpp>
#include <beluga_amcl/filter_update_control/filter_update_control_mixin.hpp>

namespace beluga_amcl
{

namespace
{

template<typename Mixin, typename Policy>
class PolicyWrapperMixin : public Mixin
{
public:
  template<typename ... Rest>
  explicit PolicyWrapperMixin(const typename Policy::param_type & config, Rest &&... rest)
  : Mixin(std::forward<Rest>(rest)...), policy_{config} {}

  [[nodiscard]] bool update_filter() {return policy_.do_resampling();}

private:
  Policy policy_;
};

using UUT = ciabatta::mixin<ciabatta::curry<PolicyWrapperMixin, ResampleIntervalPolicy>::mixin>;

struct ResampleIntervalPolicyTests : public ::testing::Test {};

TEST_F(ResampleIntervalPolicyTests, Construction) {
  // UUT does not blow up during contruction
  [[maybe_unused]] UUT uut{ResampleIntervalPolicyParam{}};
}

class ResampleIntervalPolicyTestsWithParam : public testing::TestWithParam<size_t> {};

TEST_P(ResampleIntervalPolicyTestsWithParam, ResampleEveryNthIteration) {
  // uut allows resampling every N-th iteration, starting on the N-th one
  const auto interval = GetParam();
  const auto periods = 10;

  ResampleIntervalPolicyParam config;
  config.resample_interval_count = interval;

  UUT uut{config};

  for (size_t i = 0; i < periods; ++i) {
    // don't resample for the first N-1 iterations
    for (size_t iteration = 0; iteration < interval - 1; ++iteration) {
      ASSERT_FALSE(uut.update_filter());
    }
    // then resample once
    ASSERT_TRUE(uut.update_filter());
  }
}

INSTANTIATE_TEST_SUITE_P(
  ResampleEveryNthIterationInstance,
  ResampleIntervalPolicyTestsWithParam,
  testing::Values(1, 3, 5, 10, 21));

}  // namespace

}  // namespace beluga_amcl
