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

#include <ciabatta/ciabatta.hpp>

#include <beluga/resampling_policies/resampling_policies_poller.hpp>

namespace beluga {

class ProxyVoterInterface {
 public:
  using Ptr = std::shared_ptr<ProxyVoterInterface>;
  virtual ~ProxyVoterInterface() = default;
  virtual bool current_vote_value() const = 0;
};

class MockDrivenProxyVoter : public ProxyVoterInterface {
 public:
  MOCK_METHOD(bool, current_vote_value, (), (const, override));
};

struct ProxyVoterPolicyParam {
  ProxyVoterInterface::Ptr proxy;
};

struct ProxyVoterPolicy {
  using param_type = ProxyVoterPolicyParam;

  template <class... Args>
  explicit ProxyVoterPolicy(ProxyVoterPolicyParam config) : config_{std::move(config)} {}

  template <typename Concrete>
  [[nodiscard]] bool do_resampling([[maybe_unused]] Concrete& filter) {
    return config_.proxy->current_vote_value();
  }

 private:
  ProxyVoterPolicyParam config_;
};

namespace {

struct UUT_WITH_POLICIES_INSTALLED
    : public ciabatta::mixin<
          UUT_WITH_POLICIES_INSTALLED,
          ciabatta::curry<ResamplingPoliciesPoller, ProxyVoterPolicy, ProxyVoterPolicy, ProxyVoterPolicy>::
              template mixin> {
  using ciabatta::mixin<
      UUT_WITH_POLICIES_INSTALLED,
      ciabatta::curry<ResamplingPoliciesPoller, ProxyVoterPolicy, ProxyVoterPolicy, ProxyVoterPolicy>::template mixin>::
      mixin;
};

template <class EventType>
using EventSubscriberCallback = std::function<void(const EventType&)>;

struct ResamplingPoliciesPollerTests : public ::testing::Test {};

TEST_F(ResamplingPoliciesPollerTests, ResamplingPollerWithNPoliciesUsesShortCircuitEvaluation) {
  using testing::InSequence;
  using testing::Return;

  auto voter_1 = std::make_unique<MockDrivenProxyVoter>();
  auto voter_2 = std::make_unique<MockDrivenProxyVoter>();
  auto voter_3 = std::make_unique<MockDrivenProxyVoter>();

  InSequence sec;

  // vote: false
  EXPECT_CALL(*voter_1, current_vote_value()).WillOnce(Return(false));

  // vote: false
  EXPECT_CALL(*voter_1, current_vote_value()).WillOnce(Return(true));
  EXPECT_CALL(*voter_2, current_vote_value()).WillOnce(Return(false));

  // vote: false
  EXPECT_CALL(*voter_1, current_vote_value()).WillOnce(Return(true));
  EXPECT_CALL(*voter_2, current_vote_value()).WillOnce(Return(true));
  EXPECT_CALL(*voter_3, current_vote_value()).WillOnce(Return(false));

  // vote: true
  EXPECT_CALL(*voter_1, current_vote_value()).WillOnce(Return(true));
  EXPECT_CALL(*voter_2, current_vote_value()).WillOnce(Return(true));
  EXPECT_CALL(*voter_3, current_vote_value()).WillOnce(Return(true));

  // vote: false
  EXPECT_CALL(*voter_1, current_vote_value()).WillOnce(Return(true));
  EXPECT_CALL(*voter_2, current_vote_value()).WillOnce(Return(true));
  EXPECT_CALL(*voter_3, current_vote_value()).WillOnce(Return(false));

  // vote: false
  EXPECT_CALL(*voter_1, current_vote_value()).WillOnce(Return(true));
  EXPECT_CALL(*voter_2, current_vote_value()).WillOnce(Return(false));

  // vote: false
  EXPECT_CALL(*voter_1, current_vote_value()).WillOnce(Return(false));

  UUT_WITH_POLICIES_INSTALLED uut{
      ProxyVoterPolicyParam{std::move(voter_1)}, ProxyVoterPolicyParam{std::move(voter_2)},
      ProxyVoterPolicyParam{std::move(voter_3)}};

  const auto vote_results = {
      false, /* false */
      false, /* true && false */
      false, /* true && true && false */
      true,  /* true && true && true */
      false, /* true && true && false */
      false, /* true && false */
      false, /* false */
  };

  for (const auto& iteration_result : vote_results) {
    ASSERT_EQ(iteration_result, uut.do_resampling_vote());
  }
}

TEST_F(ResamplingPoliciesPollerTests, ResamplingVoteWithNoPolicies) {
  // Tests that if no policies have been registered, the vote is by default in favour of resampling

  struct UUT_WITH_NO_POLICIES_INSTALLED
      : public ciabatta::mixin<
            UUT_WITH_NO_POLICIES_INSTALLED, ciabatta::curry<ResamplingPoliciesPoller>::template mixin> {
    using ciabatta::mixin<UUT_WITH_NO_POLICIES_INSTALLED, ciabatta::curry<ResamplingPoliciesPoller>::template mixin>::
        mixin;
  };

  UUT_WITH_NO_POLICIES_INSTALLED uut{};

  ASSERT_TRUE(uut.do_resampling_vote());
  ASSERT_TRUE(uut.do_resampling_vote());
  ASSERT_TRUE(uut.do_resampling_vote());
}

}  // namespace

}  // namespace beluga
