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

#include <utility>
#include <vector>

#include <ciabatta/ciabatta.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/const.hpp>

#include <beluga_amcl/filter_update_control/selective_resampling_policy.hpp>

namespace beluga_amcl {

namespace {

template <typename Mixin, typename Policy>
class PolicyWrapperMixin : public Mixin {
 public:
  template <typename... Rest>
  explicit PolicyWrapperMixin(const typename Policy::param_type& config, Rest&&... rest)
      : Mixin(std::forward<Rest>(rest)...), policy_{config} {}

  void set_weights(std::vector<double> v) { weights_ = std::move(v); }

  [[nodiscard]] auto weights() const { return ranges::views::all(weights_) | ranges::views::const_; }

  [[nodiscard]] bool update_filter() { return policy_.do_resampling(this->self()); }

 private:
  Policy policy_;
  std::vector<double> weights_;
};

using UUT = ciabatta::mixin<ciabatta::curry<PolicyWrapperMixin, SelectiveResamplingPolicy>::mixin>;

struct SelectiveResamplingPolicyTests : public ::testing::Test {};

TEST_F(SelectiveResamplingPolicyTests, Construction) {
  // UUT does not blow up during contruction
  [[maybe_unused]] UUT uut{SelectiveResamplingPolicyParam{}};
}

TEST_F(SelectiveResamplingPolicyTests, SelectiveResamplingOn) {
  // uut allows resampling every N-th iteration, starting on the N-th one

  SelectiveResamplingPolicyParam config;
  config.enabled = true;
  UUT uut{config};

  // very low n_eff, do resampling
  uut.set_weights({1.0, 1.0, 1.0, 1.0, 1.0});
  ASSERT_TRUE(uut.update_filter());

  // very high n_eff, don't do resampling
  uut.set_weights({0.1, 0.1, 0.1, 0.1, 0.1});
  ASSERT_FALSE(uut.update_filter());

  // n_eff slightly below N/2, do resampling
  uut.set_weights({0.2828, 0.2828, 0.2828, 0.2828, 0.30});
  ASSERT_TRUE(uut.update_filter());

  // n_eff slightly above N/2, don't do resampling
  uut.set_weights({0.2828, 0.2828, 0.2828, 0.2828, 0.25});
  ASSERT_FALSE(uut.update_filter());
}

TEST_F(SelectiveResamplingPolicyTests, SelectiveResamplingOff) {
  // uut allows resampling every N-th iteration, starting on the N-th one

  SelectiveResamplingPolicyParam config;
  config.enabled = false;
  UUT uut{config};

  // very low n_eff, do resampling
  uut.set_weights({1.0, 1.0, 1.0, 1.0, 1.0});
  ASSERT_TRUE(uut.update_filter());

  // very high n_eff, don't do resampling
  uut.set_weights({0.1, 0.1, 0.1, 0.1, 0.1});
  ASSERT_TRUE(uut.update_filter());

  // n_eff slightly below N/2, do resampling
  uut.set_weights({0.2828, 0.2828, 0.2828, 0.2828, 0.30});
  ASSERT_TRUE(uut.update_filter());

  // n_eff slightly above N/2, don't do resampling
  uut.set_weights({0.2828, 0.2828, 0.2828, 0.2828, 0.25});
  ASSERT_TRUE(uut.update_filter());
}

}  // namespace

}  // namespace beluga_amcl
