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

#include <gtest/gtest.h>

#include <beluga_estimation/beluga_estimation.hpp>

namespace {

// This test intentionally never takes the address of a registration entity.
// Including the umbrella header and querying the registry must be enough for
// the built-in registrations to have run.
TEST(BuiltinRegistryTest, UmbrellaHeaderRegistersOperations) {
  auto registry = beluga_estimation::Registry<beluga_estimation::BayesianOperation>::Instance();

  EXPECT_TRUE(registry->HasType("beluga_estimation::IMUProbe"));
  EXPECT_TRUE(registry->HasType("beluga_estimation::OdomProbe"));
  EXPECT_TRUE(registry->HasType("beluga_estimation::SE2OdometryConnector"));
  EXPECT_TRUE(registry->HasType("beluga_estimation::SE2KinematicStateConnector"));
  EXPECT_TRUE(registry->HasType("beluga_estimation::SE2PoseBroadcaster"));
  EXPECT_TRUE(registry->HasType("beluga_estimation::ExtendedKalmanFilter"));
  EXPECT_TRUE(registry->HasType("beluga_estimation::VariablePriorInitializer"));
}

TEST(BuiltinRegistryTest, UmbrellaHeaderRegistersPrograms) {
  auto registry = beluga_estimation::Registry<beluga_estimation::BayesianProgram>::Instance();

  EXPECT_TRUE(registry->HasType("beluga_estimation::RecursiveFilter"));
}

}  // namespace
