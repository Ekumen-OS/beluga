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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <beluga_estimation/core/graphviz_dumper.hpp>
#include <beluga_estimation/core/network.hpp>
#include <beluga_estimation/core/value.hpp>
#include <beluga_estimation/mocks/mock_constraint.hpp>
#include <beluga_estimation/mocks/mock_variable.hpp>

namespace {

using namespace beluga_estimation;
using ::testing::Return;

TEST(GraphvizDumperTest, DumpNetwork) {
  DynamicBayesianNetwork network;

  auto var1 = std::make_shared<MockVariable_<DefaultScalar>>(Identifier{"pose", TimePoint(std::chrono::seconds(1))});
  auto var2 = std::make_shared<MockVariable_<DefaultScalar>>(Identifier{"pose", TimePoint(std::chrono::seconds(2))});

  auto constraint =
      std::make_shared<MockConstraint_<DefaultScalar>>(Identifier{"odom", TimePoint(std::chrono::seconds(2))});

  const std::vector<Identifier> cond_vars{var1->Id()};
  const std::vector<Identifier> dep_vars{var2->Id()};

  ON_CALL(*constraint, HasConditioningVariables()).WillByDefault(Return(true));
  ON_CALL(*constraint, ConditioningVariables()).WillByDefault(Return(cond_vars));
  ON_CALL(*constraint, HasDependentVariables()).WillByDefault(Return(true));
  ON_CALL(*constraint, DependentVariables()).WillByDefault(Return(dep_vars));

  EXPECT_CALL(*constraint, HasConditioningVariables()).Times(::testing::AnyNumber());
  EXPECT_CALL(*constraint, ConditioningVariables()).Times(::testing::AnyNumber());
  EXPECT_CALL(*constraint, HasDependentVariables()).Times(::testing::AnyNumber());
  EXPECT_CALL(*constraint, DependentVariables()).Times(::testing::AnyNumber());

  auto value = std::make_shared<Value<int>>(Identifier{"measurement", TimePoint(std::chrono::seconds(1))}, 42);

  ASSERT_TRUE(network.Add(var1));
  ASSERT_TRUE(network.Add(var2));
  ASSERT_TRUE(network.Add(constraint));
  ASSERT_TRUE(network.Add(value));

  GraphvizDumper dumper;
  const std::string output = dumper.Dump(network);

  // Use the __FILE__ macro to reliably find the repository root.
  // It resolves to something like /ws/src/beluga/test/core/graphviz_dumper_test.cpp
  std::filesystem::path test_file_path = std::filesystem::absolute(__FILE__);
  std::filesystem::path repo_root = test_file_path.parent_path().parent_path().parent_path().parent_path();

  const std::filesystem::path tmp_dir = repo_root / "temp";
  std::filesystem::create_directories(tmp_dir);
  const std::filesystem::path file_path = tmp_dir / "graphviz_dumper_test.dot";

  std::ofstream out_file(file_path);
  ASSERT_TRUE(out_file.is_open()) << "Failed to open " << file_path << " for writing.";
  out_file << output;
  out_file.close();

  std::cout << "Exported Graphviz DBN to: " << file_path << "\n";

  // Basic structural checks
  EXPECT_THAT(output, ::testing::HasSubstr("digraph DBN {"));
  EXPECT_THAT(output, ::testing::HasSubstr("}"));

  // Check nodes and formatting exist
  EXPECT_THAT(output, ::testing::HasSubstr("\"pose@"));
  EXPECT_THAT(output, ::testing::HasSubstr("\"odom@"));
  EXPECT_THAT(output, ::testing::HasSubstr("\"measurement@"));
  EXPECT_THAT(output, ::testing::HasSubstr("(Variable)"));
  EXPECT_THAT(output, ::testing::HasSubstr("(Constraint)"));
  EXPECT_THAT(output, ::testing::HasSubstr("(Value)"));

  // Check edges
  EXPECT_THAT(output, ::testing::HasSubstr("->"));
}

}  // namespace
