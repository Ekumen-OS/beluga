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

#include "test_data_plugins.hpp"

#include <memory>
#include <string>
#include <utility>

#include "beluga_estimation/common/result.hpp"
#include "beluga_estimation_ros/common/registry.hpp"

namespace beluga_estimation_ros::test {

class Video : public Data {
 public:
  explicit Video(std::string prefix) : prefix_(std::move(prefix)) {}
  std::string GetLabel() const override { return prefix_ + "::video"; }

  static beluga_estimation::Result_<std::unique_ptr<Video>> Make(std::string prefix) {
    return std::make_unique<Video>(std::move(prefix));
  }

 private:
  std::string prefix_;
};

class Metadata : public Data {
 public:
  explicit Metadata(std::string prefix) : prefix_(std::move(prefix)) {}
  std::string GetLabel() const override { return prefix_ + "::metadata"; }

  static beluga_estimation::Result_<std::unique_ptr<Metadata>> Make(std::string prefix) {
    return std::make_unique<Metadata>(std::move(prefix));
  }

 private:
  std::string prefix_;
};

}  // namespace beluga_estimation_ros::test

BELUGA_ESTIMATION_ROS_REGISTER_TYPE(beluga_estimation_ros::test::Video, beluga_estimation_ros::test::Data);
BELUGA_ESTIMATION_ROS_REGISTER_TYPE(
    "Metadata",
    beluga_estimation_ros::test::Metadata,
    beluga_estimation_ros::test::Data);
