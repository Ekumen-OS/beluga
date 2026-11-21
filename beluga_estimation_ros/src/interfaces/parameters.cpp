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

#include <beluga_estimation_ros/converters/parameter_value.hpp>
#include <beluga_estimation_ros/interfaces/parameters.hpp>

namespace beluga_estimation_ros {

beluga_estimation::ParameterValue Parameters::DeclareParameter(
    std::string_view name,
    beluga_estimation::ParameterType type) {
  return FromRos(node_parameters_->declare_parameter(std::string(name), ToRos(type)));
};

beluga_estimation::ParameterValue Parameters::DeclareParameter(
    std::string_view name,
    const beluga_estimation::ParameterValue& default_value) {
  return FromRos(node_parameters_->declare_parameter(std::string(name), ToRos(default_value)));
};

beluga_estimation::ParameterValue Parameters::GetParameter(std::string_view name) const {
  return FromRos(node_parameters_->get_parameter(std::string(name)).get_parameter_value());
}

bool Parameters::HasParameter(std::string_view name) const {
  return node_parameters_->has_parameter(std::string(name));
}

}  // namespace beluga_estimation_ros
