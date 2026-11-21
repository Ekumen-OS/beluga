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

#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <bondcpp/bond.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <beluga_estimation/beluga_estimation.hpp>
#include <beluga_estimation_ros/beluga_ros.hpp>

namespace beluga_estimation_ros {

/// @brief Action the node should take after a method run completes.
enum class RunOutcomeAction {
  kDeactivate,   ///< The run finished normally; deactivate the node.
  kInterrupted,  ///< The run was interrupted; leave the node as is.
  kShutdown,     ///< The run failed; shut the node down.
};

/**
 * @brief Classifies a method run outcome into a lifecycle action.
 *
 * The node runs its method with an unlimited budget, so `kTimeout` and `kBusy`
 * can never legitimately escape `Run()`. This function demands that they do
 * not, and maps the remaining outcomes:
 * - success → @ref RunOutcomeAction::kDeactivate
 * - `kInterrupted` → @ref RunOutcomeAction::kInterrupted
 * - `kFailure` → @ref RunOutcomeAction::kShutdown
 *
 * @param outcome The result returned by the method's `Run()`.
 * @return The lifecycle action to take.
 */
RunOutcomeAction ClassifyRunOutcome(const beluga_estimation::Result& outcome);

class Node : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit Node(const rclcpp::NodeOptions& options);
  ~Node();

  using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
  std::string method_type_;
  std::unique_ptr<bond::Bond> bond_;
  std::unique_ptr<beluga_estimation::BayesianProgram> method_;
  std::optional<beluga_estimation::std_future::stop_source> stop_source_;
  std::thread method_thread_;
};

}  // namespace beluga_estimation_ros
