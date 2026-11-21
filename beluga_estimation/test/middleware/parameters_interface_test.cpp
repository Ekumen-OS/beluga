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

#include <beluga_estimation/middleware/parameters_interface.hpp>
#include <beluga_estimation/mocks/mock_parameters.hpp>

namespace {

using beluga_estimation::ParameterType;
using beluga_estimation::ParameterValue;

using testing::An;
using testing::Eq;
using testing::Return;

TEST(ParametersInterfaceTest, CanInstantiateAndCall) {
  beluga_estimation::MockParameters mock;
  beluga_estimation::ParametersInterface* ptr = &mock;

  EXPECT_CALL(mock, HasParameter("test_param")).WillOnce(Return(true));
  ASSERT_TRUE(ptr->HasParameter("test_param"));
}

TEST(ParametersInterfaceTest, TemplateDeclareWithDefault) {
  beluga_estimation::MockParameters mock;
  EXPECT_CALL(mock, DeclareParameter(Eq("test_param"), An<const ParameterValue&>()))
      .WillOnce(Return(ParameterValue(42)));

  EXPECT_EQ(mock.DeclareParameter<std::int64_t>("test_param", 10), 42);
}

TEST(ParametersInterfaceTest, TemplateDeclareWithType) {
  beluga_estimation::MockParameters mock;
  EXPECT_CALL(mock, DeclareParameter(Eq("test_param"), ParameterType::kDouble)).WillOnce(Return(ParameterValue(3.14)));

  EXPECT_DOUBLE_EQ(mock.DeclareParameter<double>("test_param"), 3.14);
}

TEST(ParametersInterfaceTest, TemplateGetParameter) {
  beluga_estimation::MockParameters mock;
  EXPECT_CALL(mock, GetParameter(Eq("test_param"))).WillOnce(Return(ParameterValue(std::string{"hello"})));

  EXPECT_EQ(mock.GetParameter<std::string>("test_param"), "hello");
}

}  // namespace
