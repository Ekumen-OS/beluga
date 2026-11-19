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
#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include <beluga_estimation/common/registry.hpp>

class Data {
 public:
  virtual ~Data() = default;
  virtual std::string Type() const = 0;
};

BELUGA_ESTIMATION_EXPORT_TYPE(Data, (std::string));

class Audio : public Data {
 public:
  explicit Audio(std::string prefix) : prefix_(std::move(prefix)) {}
  std::string Type() const override { return prefix_ + "::audio"; }

  static beluga_estimation::Result_<std::unique_ptr<Audio>> Make(std::string prefix) {
    return std::make_unique<Audio>(std::move(prefix));
  }

 private:
  std::string prefix_;
};

BELUGA_ESTIMATION_REGISTER_TYPE(Audio, Data);

class Video : public Data {
 public:
  explicit Video(std::string prefix) : prefix_(std::move(prefix)) {}
  std::string Type() const override { return prefix_ + "::video"; }

  static beluga_estimation::Result_<std::unique_ptr<Video>> Make(std::string prefix) {
    return std::make_unique<Video>(std::move(prefix));
  }

 private:
  std::string prefix_;
};

BELUGA_ESTIMATION_REGISTER_TYPE("A/V", Video, Data);

class DataMoveOnly {
 public:
  virtual ~DataMoveOnly() = default;
  virtual std::string Type() const = 0;
};

BELUGA_ESTIMATION_EXPORT_TYPE(DataMoveOnly, (std::unique_ptr<std::string>));

class ImageMoveOnly : public DataMoveOnly {
 public:
  explicit ImageMoveOnly(std::unique_ptr<std::string> prefix) : prefix_(std::move(prefix)) {}
  std::string Type() const override { return *prefix_ + "::image_move_only"; }

  static beluga_estimation::Result_<std::unique_ptr<ImageMoveOnly>> Make(std::unique_ptr<std::string> prefix) {
    return std::make_unique<ImageMoveOnly>(std::move(prefix));
  }

 private:
  std::unique_ptr<std::string> prefix_;
};

BELUGA_ESTIMATION_REGISTER_TYPE(ImageMoveOnly, DataMoveOnly);

class FailingData : public Data {
 public:
  std::string Type() const override { return "failing"; }

  static beluga_estimation::Result_<std::unique_ptr<FailingData>> Make(std::string /*prefix*/) {
    return beluga_estimation::Err(beluga_estimation::Error::kFailure, "original failure detail");
  }
};

BELUGA_ESTIMATION_REGISTER_TYPE(FailingData, Data);

namespace {

TEST(RegistryTest, HasTypeReturnsTrueForDefaultKey) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  EXPECT_TRUE(registry->HasType("Audio"));
}

TEST(RegistryTest, HasTypeReturnsTrueForExplicitKey) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  EXPECT_TRUE(registry->HasType("A/V"));
}

TEST(RegistryTest, HasTypeReturnsFalseForUnknownType) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  EXPECT_FALSE(registry->HasType("Fish"));
}

TEST(RegistryTest, HasTypeReturnsFalseForUnenrolledKey) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  EXPECT_FALSE(registry->HasType("Video"));
}

TEST(RegistryTest, MakeInstanceReturnsCorrectTypeForDefaultKey) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  auto data = registry->MakeInstance("Audio", std::string{});
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->Type(), "::audio");
}

TEST(RegistryTest, MakeInstanceTakesRvalueArguments) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  auto data = registry->MakeInstance("Audio", std::string{});
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
}

TEST(RegistryTest, MakeInstanceTakesLvalueArguments) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  std::string prefix = "canary";
  auto data = registry->MakeInstance("Audio", prefix);
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ(prefix, "canary");  // Verify original data is not corrupted
}

TEST(RegistryTest, MakeInstanceTakesConstLvalueArguments) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  const std::string prefix = "canary";
  auto data = registry->MakeInstance("Audio", prefix);
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ(prefix, "canary");  // Verify original data is not corrupted
}

TEST(RegistryTest, MakeInstanceTakesRefLvalueArguments) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  std::string prefix = "canary";
  std::string& prefix_ref = prefix;
  auto data = registry->MakeInstance("Audio", prefix_ref);
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ(prefix, "canary");  // Verify original data is not corrupted
}

TEST(RegistryTest, MakeInstanceTakesConstRefLvalueArguments) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  const std::string prefix = "canary";
  const std::string& prefix_cref = prefix;
  auto data = registry->MakeInstance("Audio", prefix_cref);
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ(prefix, "canary");  // Verify original data is not corrupted
}

TEST(RegistryTest, MakeInstanceTakesMoveOnlyArguments) {
  auto registry = beluga_estimation::Registry<DataMoveOnly>::Instance();
  auto prefix = std::make_unique<std::string>("canary");
  auto data = registry->MakeInstance("ImageMoveOnly", std::move(prefix));
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->Type(), "canary::image_move_only");
}

TEST(RegistryTest, MakeInstanceForwardsArgumentsToFactory) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  auto data = registry->MakeInstance("Audio", "raw");
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->Type(), "raw::audio");
}

TEST(RegistryTest, MakeInstanceReturnsCorrectTypeForExplicitKey) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  auto data = registry->MakeInstance("A/V", "");
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->Type(), "::video");
}

TEST(RegistryTest, MakeInstanceReturnsErrorForUnregisteredType) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  auto data = registry->MakeInstance("Fish", "");
  EXPECT_FALSE(data);
  EXPECT_EQ(data.error(), beluga_estimation::Error::kFailure);
}

TEST(RegistryTest, MakeInstancePropagatesFactoryErrorVerbatim) {
  auto registry = beluga_estimation::Registry<Data>::Instance();
  auto data = registry->MakeInstance("FailingData", "");
  ASSERT_FALSE(data);
  EXPECT_EQ(data.error(), beluga_estimation::Error::kFailure);
  EXPECT_EQ(data.error().GetDescription(), "original failure detail");
}

TEST(RegistryTest, RegisteringNewKeyInsertsTheType) {
  beluga_estimation::RegisterType<Audio, Data>("FreeKey");

  EXPECT_TRUE(beluga_estimation::Registry<Data>::Instance()->HasType("FreeKey"));
  auto data = beluga_estimation::Registry<Data>::Instance()->MakeInstance("FreeKey", std::string{});
  ASSERT_TRUE(data);
  ASSERT_NE(*data, nullptr);
  EXPECT_EQ((*data)->Type(), "::audio");
}

// A duplicate key is a programming error: gating `BELUGA_ESTIMATION_REGISTER_TYPE` behind
// `BELUGA_ESTIMATION_LINK_STATIC_REGISTRY` rules out a legitimate same-type re-registration,
// so any repetition means the registration side effect ran more than once.
TEST(RegistryTest, DuplicateRegistrationFails) {
  EXPECT_THROW((beluga_estimation::RegisterType<Audio, Data>("Audio")), std::runtime_error);
}

// Two different types claiming one key is likewise unrecoverable.
TEST(RegistryTest, ConflictingRegistrationFails) {
  EXPECT_THROW((beluga_estimation::RegisterType<Audio, Data>("A/V")), std::runtime_error);
}

}  // namespace
