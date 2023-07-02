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

#include <beluga_amcl/filter_update_control/filter_update_control_mixin.hpp>
#include <beluga_amcl/filter_update_control/update_filter_when_moving_policy.hpp>

namespace beluga_amcl {

namespace {

template <typename Mixin, typename Policy>
class PolicyWrapperMixin : public Mixin {
 public:
  using motion_event = Sophus::SE2d;

  template <typename... Rest>
  explicit PolicyWrapperMixin(const typename Policy::param_type& config, Rest&&... rest)
      : Mixin(std::forward<Rest>(rest)...), policy_{config} {}

  [[nodiscard]] bool update_filter(motion_event e) { return policy_.do_filter_update(e); }

 private:
  Policy policy_;
};

using UUT = ciabatta::mixin<ciabatta::curry<PolicyWrapperMixin, UpdateFilterWhenMovingPolicy>::mixin>;

struct ResampleOnMotionPolicyTests : public ::testing::Test {
  static auto make_motion_update_event(double x, double y, double phi) {
    using Eigen::Vector2d;
    using Sophus::SE2d;
    using Sophus::SO2d;
    return Sophus::SE2d{SO2d{phi}, Vector2d{x, y}};
  }

  const double d_threshold = 0.4;
  const double a_threshold = 0.2;
};

TEST_F(ResampleOnMotionPolicyTests, Construction) {
  // tests uut does not blow up during contruction
  [[maybe_unused]] UUT uut{UpdateFilterWhenMovingPolicyParam{}};
}

TEST_F(ResampleOnMotionPolicyTests, MotionUpdatesEnableResampling) {
  // uut allows sampling only when motion values are above parameter thresholds
  UpdateFilterWhenMovingPolicyParam params{};
  params.update_min_d = d_threshold;
  params.update_min_a = a_threshold;

  UUT uut{params};

  // first motion update, still not enough data, will default to allowing sampling
  ASSERT_TRUE(uut.update_filter(make_motion_update_event(0, 0, 0)));

  // second motion update, enough data, but we are not moving
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(0, 0, 0)));

  // still no motion
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(0, 0, 0)));

  // still no motion (thresholds are calculated independently for each axis)
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(d_threshold * 0.99, d_threshold * 0.99, a_threshold * 0.99)));

  // back to origin
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(0, 0, 0)));

  // motion above threshold for axis x
  ASSERT_TRUE(uut.update_filter(make_motion_update_event(d_threshold * 1.01, 0.0, 0.0)));

  // motion above threshold for axis y
  ASSERT_TRUE(uut.update_filter(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, 0.0)));

  // motion above threshold for angular motion
  ASSERT_TRUE(uut.update_filter(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, a_threshold * 1.01)));

  // no relative motion again
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(d_threshold * 1.01, d_threshold * 1.01, a_threshold * 1.01)));
}

TEST_F(ResampleOnMotionPolicyTests, ThresholdsAreRelativeToLatestResamplingPose) {
  // uut measures threshold from the latest pose where the policy agreed to do resampling
  UpdateFilterWhenMovingPolicyParam params{};
  params.update_min_d = d_threshold;
  params.update_min_a = a_threshold;

  UUT uut{params};

  // first motion update, still not enough data, will default to allowing sampling
  ASSERT_TRUE(uut.update_filter(make_motion_update_event(0, 0, 0)));

  // second motion update, enough data, but we are not moving
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(0, 0, 0)));

  const auto pivot_distance = d_threshold * 10.0;
  const auto lower_bound = pivot_distance - 0.95 * d_threshold;
  const auto upper_bound = pivot_distance + 0.95 * d_threshold;

  // move to 10x the threshold, forcing resampling to take place
  ASSERT_TRUE(uut.update_filter(make_motion_update_event(pivot_distance, 0.0, 0.0)));

  // move to the lower bound, then forward to the upper bound, then back to the
  // lower bound and finally back to the pivot. Despite two of the steps being
  // almost 2x the length of the threshold, we are always at a distance of less
  // than a threshold from the pivot where we last resampled, so none of the steps
  // should cause additoinal resamples value, because we never get farther away
  // from the pivot than the treshold value.
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(pivot_distance, 0.0, 0.0)));
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(lower_bound, 0.0, 0.0)));
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(upper_bound, 0.0, 0.0)));
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(lower_bound, 0.0, 0.0)));
  ASSERT_FALSE(uut.update_filter(make_motion_update_event(upper_bound, 0.0, 0.0)));

  // resample when we move a full threshold from the pivot
  ASSERT_TRUE(uut.update_filter(make_motion_update_event(pivot_distance + d_threshold * 1.01, 0.0, 0.0)));
}

TEST_F(ResampleOnMotionPolicyTests, RandomWalkingTest) {
  // simulate making the uut go on a random walk, with some steps above threshold, and others below,
  // to make sure it does not only work with the predictable poses (in particular rotations), that
  // other more systematic tests above use
  UpdateFilterWhenMovingPolicyParam params{};
  params.update_min_d = d_threshold;
  params.update_min_a = a_threshold;

  UUT uut{params};

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
  ASSERT_TRUE(uut.update_filter(current_pose));
  ASSERT_FALSE(uut.update_filter(current_pose));

  for (const auto& test_tuple : relative_motions) {
    const auto& relative_movement = std::get<0>(test_tuple);
    const auto& expected_resampling_flag = std::get<1>(test_tuple);
    // apply the relative motion on top of the current pose and test the result
    current_pose = current_pose * relative_movement;
    ASSERT_EQ(expected_resampling_flag, uut.update_filter(current_pose));
  }
}

}  // namespace

}  // namespace beluga_amcl
