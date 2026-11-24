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

#include <lifecycle_msgs/msg/state.hpp>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation_ros/node.hpp>

namespace beluga_estimation_ros {

RunOutcomeAction ClassifyRunOutcome(const beluga_estimation::Result& outcome) {
  if (outcome) {
    return RunOutcomeAction::kDeactivate;
  }
  const auto& error = outcome.error();
  // The node runs with an unlimited budget, so neither a timeout nor a busy
  // outcome can legitimately escape the method's Run().
  BELUGA_ESTIMATION_DEMAND(
      error != beluga_estimation::Error::kTimeout && error != beluga_estimation::Error::kBusy, "unexpected outcome: {}",
      error);
  if (error == beluga_estimation::Error::kInterrupted) {
    return RunOutcomeAction::kInterrupted;
  }
  return RunOutcomeAction::kShutdown;
}

Node::Node(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("beluga", options) {
  method_type_ = this->declare_parameter<std::string>("method.type", "");
  BELUGA_ESTIMATION_DEMAND(!method_type_.empty(), "Missing required parameter: method.type");
  bool managed = this->declare_parameter("managed", false);
  this->declare_parameter("heartbeat.enable", managed);
  this->declare_parameter("heartbeat.rate", 10.0);
  this->declare_parameter("heartbeat.timeout", 5.0);
  if (!managed) {
    this->configure();
    this->activate();
  }
}

Node::~Node() {
  using lifecycle_msgs::msg::State;
  if (get_current_state().id() != State::PRIMARY_STATE_FINALIZED) {
    this->shutdown();
  }
  if (method_thread_.joinable()) {
    // Thread may not have been joined yet
    // if the node was deactivated or shutdown
    // by the thread itself.
    method_thread_.join();
  }
}

Node::LifecycleNodeInterface::CallbackReturn Node::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(this->get_logger(), "Configuring %s", method_type_.c_str());
  auto registry = beluga_estimation::Registry<beluga_estimation::BayesianProgram>::Instance();
  auto middleware = std::make_shared<beluga_estimation_ros::Middleware>(this);
  auto instance_result = registry->MakeInstance(method_type_, std::move(middleware), "method.");
  if (!instance_result) {
    RCLCPP_ERROR(
        this->get_logger(), "Failed to create method instance: %s", instance_result.error().GetDescription().c_str());
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  method_ = beluga_estimation::Unwrap(instance_result);
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

Node::LifecycleNodeInterface::CallbackReturn Node::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(this->get_logger(), "Activating %s", method_type_.c_str());

  {
    stop_source_.emplace();
    if (method_thread_.joinable()) {
      // Thread may not have been joined yet
      // if the node was deactivated by the
      // thread itself.
      method_thread_.join();
    }
    method_thread_ = std::thread([this]() {
      auto outcome = method_->Run(beluga_estimation::ComputeBudget::Unlimited(stop_source_->get_token()));
      switch (ClassifyRunOutcome(outcome)) {
        case RunOutcomeAction::kShutdown:
          RCLCPP_ERROR_STREAM(this->get_logger(), "Method execution failed: " << outcome.error());
          this->shutdown();
          break;
        case RunOutcomeAction::kInterrupted:
          break;
        case RunOutcomeAction::kDeactivate:
          this->deactivate();
          break;
      }
    });
  }

  if (this->get_parameter("heartbeat.enable").as_bool()) {
    auto on_bond_formed_callback = [this]() {
      RCLCPP_INFO(get_logger(), "The bond connection to the lifecycle manager is now fully formed");
    };
    auto on_bond_broken_callback = [this]() {
      RCLCPP_ERROR(get_logger(), "The bond connection to the lifecycle manager has been broken");
    };
    bond_ = std::make_unique<bond::Bond>(
        "bond", get_name(), get_node_base_interface(), get_node_logging_interface(), get_node_parameters_interface(),
        get_node_timers_interface(), get_node_topics_interface(), on_bond_broken_callback, on_bond_formed_callback);
    bond_->setHeartbeatPeriod(1.0 / get_parameter("heartbeat.rate").as_double());
    const auto heartbeat_timeout = get_parameter("heartbeat.timeout").as_double();
    // We don't want to shorten the default connection timeout, but we want to be able to
    // make it longer because it can fail while building the likelihood map for large maps
    // just like the heartbeat timeout does
    const auto connect_timeout =
        std::max(heartbeat_timeout, static_cast<double>(bond::msg::Constants::DEFAULT_CONNECT_TIMEOUT));
    bond_->setConnectTimeout(connect_timeout);
    bond_->setHeartbeatTimeout(heartbeat_timeout);
    bond_->start();
    RCLCPP_INFO(
        get_logger(),
        "The bond (%s) connection to the lifecycle manager has been started (heartbeat timeout: %.2lf seconds)",
        get_name(), heartbeat_timeout);
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

Node::LifecycleNodeInterface::CallbackReturn Node::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(this->get_logger(), "Deactivating %s", method_type_.c_str());
  bond_.reset();
  if (std::this_thread::get_id() != method_thread_.get_id()) {
    stop_source_->request_stop();
    if (method_thread_.joinable()) {
      method_thread_.join();
    }
  }
  stop_source_.reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

Node::LifecycleNodeInterface::CallbackReturn Node::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up %s", method_type_.c_str());
  method_.reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

Node::LifecycleNodeInterface::CallbackReturn Node::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(this->get_logger(), "Shutting down %s", method_type_.c_str());
  using lifecycle_msgs::msg::State;
  if (state.id() == State::PRIMARY_STATE_ACTIVE) {
    bond_.reset();
    if (std::this_thread::get_id() != method_thread_.get_id()) {
      stop_source_->request_stop();
      if (method_thread_.joinable()) {
        method_thread_.join();
      }
    }
    stop_source_.reset();
  }
  method_.reset();
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace beluga_estimation_ros

BELUGA_ESTIMATION_ROS_IMPORT_TYPE(beluga_estimation::BayesianProgram);
BELUGA_ESTIMATION_ROS_IMPORT_TYPE(beluga_estimation::BayesianOperation);

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(beluga_estimation_ros::Node)
