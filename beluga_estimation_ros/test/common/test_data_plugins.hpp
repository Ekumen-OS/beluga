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

#pragma once

#include <string>

#include "beluga_estimation/common/registry.hpp"
#include "beluga_estimation_ros/common/registry.hpp"

namespace beluga_estimation_ros::test {

class Data {
 public:
  virtual ~Data() = default;
  virtual std::string GetLabel() const = 0;
};

}  // namespace beluga_estimation_ros::test

BELUGA_ESTIMATION_EXPORT_TYPE(beluga_estimation_ros::test::Data, (std::string));
