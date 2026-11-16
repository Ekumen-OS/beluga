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

#include <gmock/gmock.h>

#include <beluga_estimation/middleware/parameter_value.hpp>

namespace {

using beluga_estimation::ParameterType;
using beluga_estimation::ParameterTypeException;
using beluga_estimation::ParameterValue;

TEST(ParameterValueTest, DefaultConstructor) {
  const ParameterValue value;
  EXPECT_EQ(value.GetType(), ParameterType::kNotSet);
  EXPECT_EQ(value.GetTypeName(), "NOT_SET");
}

TEST(ParameterValueTest, BoolValue) {
  const ParameterValue value{true};
  EXPECT_EQ(value.GetType(), ParameterType::kBool);
  EXPECT_EQ(value.GetTypeName(), "BOOL");
  EXPECT_TRUE(value.AsBool());
  EXPECT_TRUE(value.As<bool>());
  EXPECT_TRUE((value.As<ParameterType::kBool>()));
}

TEST(ParameterValueTest, IntValue) {
  const ParameterValue value{std::int64_t{42}};
  EXPECT_EQ(value.GetType(), ParameterType::kInteger);
  EXPECT_EQ(value.AsInt(), 42);
  EXPECT_EQ(value.As<std::int64_t>(), 42);
}

TEST(ParameterValueTest, DoubleValue) {
  const ParameterValue value{3.14};
  EXPECT_EQ(value.GetType(), ParameterType::kDouble);
  EXPECT_DOUBLE_EQ(value.AsDouble(), 3.14);
}

TEST(ParameterValueTest, StringValue) {
  const ParameterValue value{std::string{"hello"}};
  EXPECT_EQ(value.GetType(), ParameterType::kString);
  EXPECT_EQ(value.AsString(), "hello");
}

TEST(ParameterValueTest, StringViewValue) {
  const ParameterValue value{std::string_view{"hello"}};
  EXPECT_EQ(value.GetType(), ParameterType::kString);
  EXPECT_EQ(value.AsString(), "hello");
}

TEST(ParameterValueTest, StringLiteralValue) {
  const ParameterValue value{"hello"};
  EXPECT_EQ(value.GetType(), ParameterType::kString);
  EXPECT_EQ(value.AsString(), "hello");
}

TEST(ParameterValueTest, BoolArrayValue) {
  const std::vector<bool> data{true, false};
  const ParameterValue value{data};
  EXPECT_EQ(value.GetType(), ParameterType::kBoolArray);
  EXPECT_EQ(value.AsBoolArray(), data);
}

TEST(ParameterValueTest, IntArrayValue) {
  const std::vector<std::int64_t> data{1, 2, 3};
  const ParameterValue value{data};
  EXPECT_EQ(value.GetType(), ParameterType::kIntegerArray);
  EXPECT_EQ(value.AsIntArray(), data);
}

TEST(ParameterValueTest, DoubleArrayValue) {
  const std::vector<double> data{1.1, 2.2};
  const ParameterValue value{data};
  EXPECT_EQ(value.GetType(), ParameterType::kDoubleArray);
  EXPECT_EQ(value.AsDoubleArray(), data);
}

TEST(ParameterValueTest, StringArrayValue) {
  const std::vector<std::string> data{"a", "b"};
  const ParameterValue value{data};
  EXPECT_EQ(value.GetType(), ParameterType::kStringArray);
  EXPECT_EQ(value.AsStringArray(), data);
}

TEST(ParameterValueTest, TypeMismatchThrows) {
  const ParameterValue value{42};
  EXPECT_THROW(value.AsBool(), ParameterTypeException);
  EXPECT_THROW(value.AsDouble(), ParameterTypeException);
  EXPECT_THROW(value.AsString(), ParameterTypeException);
  EXPECT_THROW((value.As<ParameterType::kBool>()), ParameterTypeException);
}

TEST(ParameterValueTest, ToStringFunctions) {
  EXPECT_EQ(beluga_estimation::ToString(ParameterType::kDouble), "DOUBLE");
  EXPECT_EQ(beluga_estimation::ToStringView(ParameterType::kInteger), "INTEGER");
}

TEST(ParameterValueTest, ExceptionMessage) {
  try {
    const ParameterValue value{true};
    [[maybe_unused]] auto _ = value.AsInt();
    FAIL() << "Should have thrown ParameterTypeException";
  } catch (const ParameterTypeException& e) {
    EXPECT_THAT(e.what(), testing::HasSubstr("BOOL"));
    EXPECT_THAT(e.what(), testing::HasSubstr("INTEGER"));
  }
}

}  // namespace
