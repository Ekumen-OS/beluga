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

#include <beluga/resampling_policies/resample_on_motion_policy.hpp>
#include <beluga/resampling_policies/resampling_policies_poller.hpp>

namespace beluga {

namespace {

template <class Mixin>
struct MockMotionModel : public Mixin {
  using update_type = Sophus::SE2d;

  template <class... Args>
  explicit MockMotionModel(Args&&... rest) : Mixin(std::forward<Args>(rest)...) {}

  void update_motion(const update_type& pose) { last_pose_ = pose; }
  [[nodiscard]] std::optional<update_type> latest_motion_update() const { return last_pose_; }

 private:
  std::optional<update_type> last_pose_;
};

struct UUT : public ciabatta::mixin<
                 UUT,
                 MockMotionModel,
                 ciabatta::curry<ResamplingPoliciesPoller, ResampleOnMotionPolicy>::template mixin> {
  using ciabatta::mixin<
      UUT,
      MockMotionModel,
      ciabatta::curry<ResamplingPoliciesPoller, ResampleOnMotionPolicy>::template mixin>::mixin;
};

struct ResampleOnMotionPolicyTests : public ::testing::Test {
  auto make_motion_update_event(double x, double y, double phi) {
    using Eigen::Vector2d;
    using Sophus::SE2d;
    using Sophus::SO2d;
    return Sophus::SE2d{SO2d{phi}, Vector2d{x, y}};
  };

  const double d_threshold = 0.4;
  const double a_threshold = 0.2;
};

TEST_F(ResampleOnMotionPolicyTests, Construction) {
  // tests uut does not blow up during contruction
  [[maybe_unused]] UUT uut{ResampleOnMotionPolicyParam{}};
}

TEST_F(ResampleOnMotionPolicyTests, MotionUpdatesEnableResampling) {
  // uut allows sampling only when motion values are above parameter thresholds
  ResampleOnMotionPolicyParam params{};
  params.update_min_d = d_threshold;
  params.update_min_a = a_threshold;

  UUT uut{params};

  // when there's no motion data, policy should default to true
  ASSERT_TRUE(uut.do_resampling_vote());

  // first motion update, still not enough data, will default to allowing sampling
  uut.update_motion(make_motion_update_event(0, 0, 0));
  ASSERT_TRUE(uut.do_resampling_vote());

  // second motion update, enough data, but we are not moving
  uut.update_motion(make_motion_update_event(0, 0, 0));
  ASSERT_FALSE(uut.do_resampling_vote());

  // still no motion
  uut.update_motion(make_motion_update_event(0, 0, 0));
  ASSERT_FALSE(uut.do_resampling_vote());

  // still no motion (thresholds are calculated independently for each axis)
  uut.update_motion(make_motion_update_event(d_threshold * 0.99, d_threshold * 0.99, a_threshold * 0.99));
  ASSERT_FALSE(uut.do_resampling_vote());

  // back to origin
  uut.update_motion(make_motion_update_event(0, 0, 0));

  // motion above threshold for axis x
  uut.update_motion(make_motion_update_event(d_threshold * 1.01, 0.0, 0.0));
  ASSERT_TRUE(uut.do_resampling_vote());

  // motion above threshold for axis y
  uut.update_motion(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, 0.0));
  ASSERT_TRUE(uut.do_resampling_vote());

  // motion above threshold for angular motion
  uut.update_motion(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, a_threshold * 1.01));
  ASSERT_TRUE(uut.do_resampling_vote());

  // no relative motion again
  uut.update_motion(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, a_threshold * 1.01));
  ASSERT_FALSE(uut.do_resampling_vote());
}

TEST_F(ResampleOnMotionPolicyTests, ThresholdsAreRelativeToLatestResamplingPose) {
  // uut measures threshold from the latest pose where the policy agreed to do resampling
  ResampleOnMotionPolicyParam params{};
  params.update_min_d = d_threshold;
  params.update_min_a = a_threshold;

  UUT uut{params};

  // when there's no motion data, policy should default to true
  ASSERT_TRUE(uut.do_resampling_vote());

  // first motion update, still not enough data, will default to allowing sampling
  uut.update_motion(make_motion_update_event(0, 0, 0));
  ASSERT_TRUE(uut.do_resampling_vote());

  // second motion update, enough data, but we are not moving
  uut.update_motion(make_motion_update_event(0, 0, 0));
  ASSERT_FALSE(uut.do_resampling_vote());

  const auto pivot_distance = d_threshold * 10.0;
  const auto lower_bound = pivot_distance - 0.95 * d_threshold;
  const auto upper_bound = pivot_distance + 0.95 * d_threshold;

  // move to 10x the threshold, forcing resampling to take place
  uut.update_motion(make_motion_update_event(pivot_distance, 0.0, 0.0));
  ASSERT_TRUE(uut.do_resampling_vote());

  // move to the lower bound, then forward to the upper bound, then back to the lower bound and finally back to the
  // pivot. Despite two of the steps being almost 2x the length of the threshold, we are always at a distance of less
  // than a threshold from the pivot where we last resampled, so none of the steps should cause additoinal resamples
  // value, because we never get farther away from the pivot than the treshold value.
  uut.update_motion(make_motion_update_event(pivot_distance, 0.0, 0.0));
  ASSERT_FALSE(uut.do_resampling_vote());
  uut.update_motion(make_motion_update_event(lower_bound, 0.0, 0.0));
  ASSERT_FALSE(uut.do_resampling_vote());
  uut.update_motion(make_motion_update_event(upper_bound, 0.0, 0.0));
  ASSERT_FALSE(uut.do_resampling_vote());
  uut.update_motion(make_motion_update_event(lower_bound, 0.0, 0.0));
  ASSERT_FALSE(uut.do_resampling_vote());
  uut.update_motion(make_motion_update_event(upper_bound, 0.0, 0.0));
  ASSERT_FALSE(uut.do_resampling_vote());

  // resample when we move a full threshold from the pivot
  uut.update_motion(make_motion_update_event(pivot_distance + d_threshold * 1.01, 0.0, 0.0));
  ASSERT_TRUE(uut.do_resampling_vote());
}

TEST_F(ResampleOnMotionPolicyTests, RandomWalkingTest) {
  // simulate making the uut go on a random walk, with some steps above threshold, and others below,
  // to make sure it does not only work with the predictable poses (in particular rotations), that
  // other more systematic tests above use
  ResampleOnMotionPolicyParam params{};
  params.update_min_d = d_threshold;
  params.update_min_a = a_threshold;

  UUT uut{params};

  // when there's no motion data, policy should default to true
  ASSERT_TRUE(uut.do_resampling_vote());

  // list of pairs of relative motion from the previous pose, and the expected
  // "do_resampling" value for that step
  const auto relative_motions = {
      std::make_tuple(make_motion_update_event(0, 0, 0), false),
      // relative motions along x axis
      std::make_tuple(make_motion_update_event(d_threshold * 0.99, 0.0, 0.0), false),
      std::make_tuple(make_motion_update_event(d_threshold * 0.02, 0.0, 0.0), true),
      std::make_tuple(make_motion_update_event(d_threshold * -0.99, 0.0, 0.0), false),
      std::make_tuple(make_motion_update_event(d_threshold * -0.02, 0.0, 0.0), true),
      // relative motions along y axis
      std::make_tuple(make_motion_update_event(0.0, d_threshold * 0.99, 0.0), false),
      std::make_tuple(make_motion_update_event(0.0, d_threshold * 0.02, 0.0), true),
      std::make_tuple(make_motion_update_event(0.0, d_threshold * -0.99, 0.0), false),
      std::make_tuple(make_motion_update_event(0.0, d_threshold * -0.02, 0.0), true),
      // relative motions along theta axis
      std::make_tuple(make_motion_update_event(0.0, 0.0, a_threshold * 0.99), false),
      std::make_tuple(make_motion_update_event(0.0, 0.0, a_threshold * 0.02), true),
      std::make_tuple(make_motion_update_event(0.0, 0.0, a_threshold * -0.99), false),
      std::make_tuple(make_motion_update_event(0.0, 0.0, a_threshold * -0.02), true),
      // random walk move and stop 1
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 0.99, a_threshold * 0.99), true),
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, a_threshold * 0.40), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.20, d_threshold * 0.10, a_threshold * 1.01), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.99, d_threshold * 0.99, a_threshold * 0.99), false),
      // random walk move and stop 2
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 0.30, a_threshold * 0.10), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.50, d_threshold * 1.01, a_threshold * 0.30), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.70, d_threshold * 0.40, a_threshold * 1.01), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.99, d_threshold * 0.99, a_threshold * 0.99), false),
      // random walk move and stop 3
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 0.99, a_threshold * 0.99), true),
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, a_threshold * 0.70), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.30, d_threshold * 0.40, a_threshold * 1.01), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.01, d_threshold * 0.99, a_threshold * 0.99), false),
      // random walk move and stop 4
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 0.99, a_threshold * 0.99), true),
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 0.80, a_threshold * 0.10), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.20, d_threshold * 0.50, a_threshold * 1.01), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.99, d_threshold * 0.99, a_threshold * 0.99), false),
      // random walk move and stop 5
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 0.30, a_threshold * 0.10), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.50, d_threshold * 1.01, a_threshold * 0.80), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.20, d_threshold * 0.40, a_threshold * 1.01), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.99, d_threshold * 0.99, a_threshold * 0.99), false),
      // random walk move and stop 6
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 0.99, a_threshold * 0.99), true),
      std::make_tuple(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, a_threshold * 0.50), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.40, d_threshold * 0.70, a_threshold * 1.01), true),
      std::make_tuple(make_motion_update_event(d_threshold * 0.99, d_threshold * 0.99, a_threshold * 0.99), false),
  };

  auto current_pose = make_motion_update_event(0, 0, 0);

  // warm-up the internal state of the policy
  uut.update_motion(current_pose);
  ASSERT_TRUE(uut.do_resampling_vote());
  uut.update_motion(current_pose);
  ASSERT_FALSE(uut.do_resampling_vote());

  for (const auto& test_tuple : relative_motions) {
    const auto relative_movement = std::get<0>(test_tuple);
    const auto expected_resampling_flag = std::get<1>(test_tuple);
    // apply the relative motion on top of the current pose and test the result
    current_pose = current_pose * relative_movement;
    uut.update_motion(current_pose);
    ASSERT_EQ(expected_resampling_flag, uut.do_resampling_vote());
  }
}

}  // namespace

}  // namespace beluga
