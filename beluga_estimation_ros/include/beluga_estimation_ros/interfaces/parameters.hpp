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

#include <rclcpp/node_interfaces/node_parameters.hpp>

#include <beluga_estimation/middleware/parameters_interface.hpp>

namespace beluga_estimation_ros {

/**
 * @brief ROS 2 implementation of the Beluga parameters interface.
 *
 * This class wraps a ROS 2 node parameters interface to provide parameter
 * management using ROS 2 as the backend.
 */
class Parameters : public beluga_estimation::ParametersInterface {
 public:
  /**
   * @brief Constructs a Parameters instance.
   * @param node_parameters The ROS 2 node parameters interface to wrap.
   */
  explicit Parameters(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters)
      : node_parameters_(node_parameters) {}

  using beluga_estimation::ParametersInterface::DeclareParameter;
  using beluga_estimation::ParametersInterface::GetParameter;

  /// \copydoc beluga_estimation::ParametersInterface::DeclareParameter(std::string_view,
  /// beluga_estimation::ParameterType)
  beluga_estimation::ParameterValue DeclareParameter(std::string_view, beluga_estimation::ParameterType) override;

  /// \copydoc beluga_estimation::ParametersInterface::DeclareParameter(std::string_view, const
  /// beluga_estimation::ParameterValue&)
  beluga_estimation::ParameterValue DeclareParameter(std::string_view, const beluga_estimation::ParameterValue&)
      override;

  /// \copydoc beluga_estimation::ParametersInterface::GetParameter(std::string_view) const
  beluga_estimation::ParameterValue GetParameter(std::string_view) const override;

  /// \copydoc beluga_estimation::ParametersInterface::HasParameter(std::string_view) const
  bool HasParameter(std::string_view) const override;

 private:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
};

}  // namespace beluga_estimation_ros
