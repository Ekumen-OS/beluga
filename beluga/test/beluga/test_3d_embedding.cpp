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

#include <gtest/gtest.h>
#include <beluga/3d_embedding.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

using Sophus::SE2d;
using Sophus::SE3d;
using Sophus::SO2d;
using Sophus::SO3d;
namespace beluga {

TEST(Embed3DTests, Se3ToSe2) {
  {
    const auto identity_2d = SE2d{};
    const auto identity_3d = SE3d{};
    ASSERT_TRUE(identity_2d.matrix().isApprox(To2d(identity_3d).matrix()));
  }
  {
    const auto rotation_about_z_2d = SE2d::rot(0.5);
    const auto rotation_about_z_3d = SE3d::rotZ(0.5);
    ASSERT_TRUE(rotation_about_z_2d.matrix().isApprox(To2d(rotation_about_z_3d).matrix()));
  }
  {
    const auto identity_2d = SE2d{};
    const auto rotation_about_x_3d = SE3d::rotX(0.5);
    ASSERT_TRUE(identity_2d.matrix().isApprox(To2d(rotation_about_x_3d).matrix()));
  }
  {
    const auto identity_2d = SE2d{};
    const auto rotation_about_y_3d = SE3d::rotY(0.5);
    ASSERT_TRUE(identity_2d.matrix().isApprox(To2d(rotation_about_y_3d).matrix()));
  }
  {
    const auto translation_3d = SE3d{Sophus::SO3d{}, Eigen::Vector3d{1, 2, 4}};
    ASSERT_TRUE(To2d(translation_3d).translation().isApprox(Eigen::Vector2d{1, 2}));
  }
}

TEST(Embed3DTests, Se2ToSe3) {
  {
    const auto identity_3d = SE3d{};
    const auto identity_2d = SE2d{};
    ASSERT_TRUE(To3d(identity_2d).matrix().isApprox(identity_3d.matrix()));
  }
  {
    const auto rotation_about_z_3d = SE3d::rotZ(0.5);
    const auto rotation_about_z_2d = SE2d::rot(0.5);
    ASSERT_TRUE(To3d(rotation_about_z_2d).matrix().isApprox(rotation_about_z_3d.matrix()));
  }
  {
    const auto translation_2d = SE2d{0, Eigen::Vector2d{1, 2}};
    ASSERT_TRUE(To3d(translation_2d).translation().isApprox(Eigen::Vector3d{1, 2, 0}));
  }
}

}  // namespace beluga
