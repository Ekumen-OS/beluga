// Copyright 2022-2023 Ekumen, Inc.
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

#include <beluga/algorithm/exponential_filter.hpp>

namespace {

TEST(ExponentialFilter, Update) {
  auto filter = beluga::ExponentialFilter{0.1};
  ASSERT_NEAR(1.000, filter(1), 0.00001);
  ASSERT_NEAR(1.100, filter(2), 0.00001);
  ASSERT_NEAR(1.290, filter(3), 0.00001);
  ASSERT_NEAR(1.161, filter(0), 0.00001);
}

TEST(ExponentialFilter, Passthrough) {
  auto filter = beluga::ExponentialFilter{1.};
  ASSERT_NEAR(1.0, filter(1), 0.00001);
  ASSERT_NEAR(2.0, filter(2), 0.00001);
  ASSERT_NEAR(3.0, filter(3), 0.00001);
  ASSERT_NEAR(0.0, filter(0), 0.00001);
}

TEST(ExponentialFilter, Reset) {
  auto filter = beluga::ExponentialFilter{0.1};
  ASSERT_NEAR(1.0, filter(1), 0.00001);
  ASSERT_NEAR(1.1, filter(2), 0.00001);
  filter.reset();
  ASSERT_NEAR(3.0, filter(3), 0.00001);
}

}  // namespace
