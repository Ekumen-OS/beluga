// Copyright 2026 Ekumen, Inc.
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

#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation_ros/common/registry.hpp>
#include "test_data_plugins.hpp"

namespace beluga_estimation_ros::test {

class Audio : public Data {
 public:
  explicit Audio(std::string prefix) : prefix_(std::move(prefix)) {}
  std::string GetLabel() const override { return prefix_ + "::audio"; }

  static beluga_estimation::Result_<std::unique_ptr<Audio>> Make(std::string prefix) {
    return std::make_unique<Audio>(std::move(prefix));
  }

 private:
  std::string prefix_;
};

}  // namespace beluga_estimation_ros::test

BELUGA_ESTIMATION_REGISTER_TYPE(beluga_estimation_ros::test::Audio, beluga_estimation_ros::test::Data);

BELUGA_ESTIMATION_ROS_IMPORT_TYPE(beluga_estimation_ros::test::Data);

namespace {

TEST(RegistryTest, HasTypeReturnsTrueForLocalType) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  EXPECT_TRUE(registry->HasType("beluga_estimation_ros::test::Audio"));
}

TEST(RegistryTest, HasTypeReturnsTrueForDefaultKey) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  EXPECT_TRUE(registry->HasType("beluga_estimation_ros::test::Video"));
}

TEST(RegistryTest, HasTypeReturnsTrueForExplicitKey) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  EXPECT_TRUE(registry->HasType("Metadata"));
}

TEST(RegistryTest, HasTypeReturnsFalseForUnknownType) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  EXPECT_FALSE(registry->HasType("NotAPlugin"));
}

TEST(RegistryTest, HasTypeReturnsFalseForUnenrolledKey) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  EXPECT_FALSE(registry->HasType("beluga_estimation_ros::test::Metadata"));
}

TEST(RegistryTest, MakeInstanceReturnsCorrectTypeForLocalType) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  auto data = registry->MakeInstance("beluga_estimation_ros::test::Audio", std::string{});
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->GetLabel(), "::audio");
}

TEST(RegistryTest, MakeInstanceReturnsCorrectTypeForDefaultKey) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  auto data = registry->MakeInstance("beluga_estimation_ros::test::Video", std::string{});
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->GetLabel(), "::video");
}

TEST(RegistryTest, MakeInstanceForwardsArgumentsToFactory) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  auto data = registry->MakeInstance("beluga_estimation_ros::test::Video", "raw");
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->GetLabel(), "raw::video");
}

TEST(RegistryTest, MakeInstanceReturnsCorrectTypeForExplicitKey) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  auto data = registry->MakeInstance("Metadata", "");
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->GetLabel(), "::metadata");
}

TEST(RegistryTest, MakeInstanceReturnsErrorForUnregisteredType) {
  auto registry = beluga_estimation::Registry<beluga_estimation_ros::test::Data>::Instance();
  auto data = registry->MakeInstance("NotAPlugin", "");
  EXPECT_FALSE(data);
  EXPECT_EQ(data.error(), beluga_estimation::Error::kFailure);
}

}  // namespace
