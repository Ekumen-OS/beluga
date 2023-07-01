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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include <ciabatta/ciabatta.hpp>

#include <beluga_amcl/filter_update_control/filter_update_control_mixin.hpp>

using ::testing::_;
using ::testing::StrictMock;

namespace beluga_amcl {

using motion_event_type = Sophus::SE2d;
using laser_scan_type = std::vector<std::pair<double, double>>;

template <typename T>
struct ProxyVoterPolicyParam {
  std::shared_ptr<T> proxy;
};

struct ResampleIntervalPolicyMock {
  MOCK_METHOD(bool, do_resampling, (), ());
};

struct SelectiveResamplingPolicyMock {
  MOCK_METHOD(bool, do_resampling, (), ());
};

struct UpdateFilterWhenMovingPolicyMock {
  MOCK_METHOD(bool, do_filter_update, (const motion_event_type& current_pose_in_odom), (const));
};

struct ResampleIntervalPolicyProxyPolicy {
  using param_type = ProxyVoterPolicyParam<ResampleIntervalPolicyMock>;

  explicit ResampleIntervalPolicyProxyPolicy(param_type config) : config_{std::move(config)} {}

  [[nodiscard]] bool do_resampling() { return config_.proxy->do_resampling(); }

 private:
  param_type config_;
};

struct SelectiveResamplingProxyPolicy {
  using param_type = ProxyVoterPolicyParam<SelectiveResamplingPolicyMock>;

  explicit SelectiveResamplingProxyPolicy(param_type config) : config_{std::move(config)} {}

  // selective resampling interface
  template <typename Concrete>
  [[nodiscard]] bool do_resampling([[maybe_unused]] Concrete& filter) {
    return config_.proxy->do_resampling();
  }

 private:
  param_type config_;
};

struct UpdateFilterWhenMovingProxyPolicy {
  using param_type = ProxyVoterPolicyParam<UpdateFilterWhenMovingPolicyMock>;

  explicit UpdateFilterWhenMovingProxyPolicy(param_type config) : config_{std::move(config)} {}

  [[nodiscard]] bool do_filter_update(const motion_event_type& pose) { return config_.proxy->do_filter_update(pose); }

 private:
  param_type config_;
};

template <typename Mixin>
class MockMixin : public Mixin {
 public:
  template <typename... Args>
  explicit MockMixin(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  MOCK_METHOD(void, update_motion, (motion_event_type));
  MOCK_METHOD(void, sample, ());
  MOCK_METHOD(void, update_sensor, (laser_scan_type));
  MOCK_METHOD(void, reweight, ());
  MOCK_METHOD(void, resample, ());

  void sample(std::execution::sequenced_policy) { sample(); }
  void reweight(std::execution::sequenced_policy) { reweight(); }
  void sample(std::execution::parallel_policy) { sample(); }
  void reweight(std::execution::parallel_policy) { reweight(); }
};

namespace {

using UUT_WITH_POLICIES_INSTALLED = StrictMock<ciabatta::mixin<
    MockMixin,
    ciabatta::curry<
        FilterUpdateControlMixin,
        UpdateFilterWhenMovingProxyPolicy,
        ResampleIntervalPolicyProxyPolicy,
        SelectiveResamplingProxyPolicy>::mixin,
    ciabatta::provides<FilterUpdateControlInterface>::template mixin>>;

template <class EventType>
using EventSubscriberCallback = std::function<void(const EventType&)>;

struct FilterUpdateControlTests : public ::testing::Test {};

TEST_F(FilterUpdateControlTests, ResamplingPollerWithNPoliciesUsesShortCircuitEvaluation) {
  using testing::InSequence;
  using testing::Return;

  auto motion_policy_result = std::make_unique<UpdateFilterWhenMovingPolicyMock>();
  auto resample_rate_divider_result = std::make_unique<ResampleIntervalPolicyMock>();
  auto selective_resampler_policy_result = std::make_unique<SelectiveResamplingPolicyMock>();

  std::vector<bool> expected_return_values;

  {
    InSequence sec;

    // iteration 1
    // no estimation update
    EXPECT_CALL(*motion_policy_result, do_filter_update(_)).WillOnce(Return(false));
    expected_return_values.push_back(false);

    // iteration 2
    // no estimation update
    EXPECT_CALL(*motion_policy_result, do_filter_update(_)).WillOnce(Return(true));
    EXPECT_CALL(*resample_rate_divider_result, do_resampling()).WillOnce(Return(false));
    expected_return_values.push_back(false);

    // iteration 3
    // estimation update (without resampling)
    EXPECT_CALL(*motion_policy_result, do_filter_update(_)).WillOnce(Return(true));
    EXPECT_CALL(*resample_rate_divider_result, do_resampling()).WillOnce(Return(true));
    EXPECT_CALL(*selective_resampler_policy_result, do_resampling()).WillOnce(Return(false));
    expected_return_values.push_back(true);

    // iteration 4
    // estimation update (with resampling)
    EXPECT_CALL(*motion_policy_result, do_filter_update(_)).WillOnce(Return(true));
    EXPECT_CALL(*resample_rate_divider_result, do_resampling()).WillOnce(Return(true));
    EXPECT_CALL(*selective_resampler_policy_result, do_resampling()).WillOnce(Return(true));
    expected_return_values.push_back(true);

    // iteration 5
    // no estimation update
    EXPECT_CALL(*motion_policy_result, do_filter_update(_)).WillOnce(Return(false));
    expected_return_values.push_back(false);

    // iteration 6
    // no estimation update
    EXPECT_CALL(*motion_policy_result, do_filter_update(_)).WillOnce(Return(true));
    EXPECT_CALL(*resample_rate_divider_result, do_resampling()).WillOnce(Return(false));
    expected_return_values.push_back(false);
  }

  [[maybe_unused]] UUT_WITH_POLICIES_INSTALLED uut{
      UpdateFilterWhenMovingProxyPolicy::param_type{std::move(motion_policy_result)},
      ResampleIntervalPolicyProxyPolicy::param_type{std::move(resample_rate_divider_result)},
      SelectiveResamplingProxyPolicy::param_type{std::move(selective_resampler_policy_result)}};

  {
    InSequence sec;
    // iteration 1
    // no calls on the filter

    // iteration 2
    EXPECT_CALL(uut, update_motion(_)).Times(1);
    EXPECT_CALL(uut, sample()).Times(1);
    EXPECT_CALL(uut, update_sensor(_)).Times(1);
    EXPECT_CALL(uut, reweight()).Times(1);

    // iteration 3
    EXPECT_CALL(uut, update_motion(_)).Times(1);
    EXPECT_CALL(uut, sample()).Times(1);
    EXPECT_CALL(uut, update_sensor(_)).Times(1);
    EXPECT_CALL(uut, reweight()).Times(1);

    // iteration 4
    EXPECT_CALL(uut, update_motion(_)).Times(1);
    EXPECT_CALL(uut, sample()).Times(1);
    EXPECT_CALL(uut, update_sensor(_)).Times(1);
    EXPECT_CALL(uut, reweight()).Times(1);
    EXPECT_CALL(uut, resample()).Times(1);

    // iteration 5
    // no calls on the filter

    // iteration 6
    EXPECT_CALL(uut, update_motion(_)).Times(1);
    EXPECT_CALL(uut, sample()).Times(1);
    EXPECT_CALL(uut, update_sensor(_)).Times(1);
    EXPECT_CALL(uut, reweight()).Times(1);
  }

  for (const auto& iteration_result : expected_return_values) {
    ASSERT_EQ(iteration_result, uut.update_filter(motion_event_type{}, laser_scan_type{}));
  }
}

}  // namespace

}  // namespace beluga_amcl
