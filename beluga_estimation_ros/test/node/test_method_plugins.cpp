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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <beluga_estimation/beluga_estimation.hpp>
#include <beluga_estimation_ros/beluga_ros.hpp>

namespace beluga_estimation_ros::test {

/// @brief Mock method that completes immediately with success.
class MockMethodQuick : public beluga_estimation::BayesianProgram {
 public:
  explicit MockMethodQuick(std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware, std::string_view prefix)
      : middleware_(std::move(middleware)), prefix_(prefix) {}

  static beluga_estimation::Result_<std::unique_ptr<MockMethodQuick>> Make(
      std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<MockMethodQuick>(std::move(middleware), prefix);
  }

  beluga_estimation::Result Run(const beluga_estimation::ComputeBudget&) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    run_count_.fetch_add(1, std::memory_order_relaxed);
    return beluga_estimation::Ok();
  }

  int GetRunCount() const { return run_count_.load(std::memory_order_relaxed); }

 private:
  std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware_;
  std::string prefix_;
  std::atomic<int> run_count_{0};
};

/// @brief Mock method that runs slowly and respects stop tokens.
class MockMethodSlow : public beluga_estimation::BayesianProgram {
 public:
  explicit MockMethodSlow(std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware, std::string_view prefix)
      : middleware_(std::move(middleware)), prefix_(prefix) {}

  static beluga_estimation::Result_<std::unique_ptr<MockMethodSlow>> Make(
      std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<MockMethodSlow>(std::move(middleware), prefix);
  }

  beluga_estimation::Result Run(const beluga_estimation::ComputeBudget& budget) override {
    run_count_.fetch_add(1, std::memory_order_relaxed);
    started_.store(true, std::memory_order_release);

    // Run for up to 10 seconds or until stop is requested
    constexpr auto kMaxDuration = std::chrono::seconds(10);
    const auto start_time = std::chrono::system_clock::now();

    while (std::chrono::system_clock::now() - start_time < kMaxDuration) {
      // Check budget for interruption
      auto result = budget.Check(std::chrono::system_clock::now());
      if (!result) {
        completed_.store(true, std::memory_order_release);
        return result;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    completed_.store(true, std::memory_order_release);
    return beluga_estimation::Ok();
  }

  int GetRunCount() const { return run_count_.load(std::memory_order_relaxed); }
  bool HasStarted() const { return started_.load(std::memory_order_acquire); }
  bool HasCompleted() const { return completed_.load(std::memory_order_acquire); }

 private:
  std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware_;
  std::string prefix_;
  std::atomic<int> run_count_{0};
  std::atomic<bool> started_{false};
  std::atomic<bool> completed_{false};
};

/// @brief Mock method that immediately returns an error.
class MockMethodFailing : public beluga_estimation::BayesianProgram {
 public:
  explicit MockMethodFailing(
      std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware,
      std::string_view prefix)
      : middleware_(std::move(middleware)), prefix_(prefix) {}

  static beluga_estimation::Result_<std::unique_ptr<MockMethodFailing>> Make(
      std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware,
      std::string_view prefix) {
    return std::make_unique<MockMethodFailing>(std::move(middleware), prefix);
  }

  beluga_estimation::Result Run(const beluga_estimation::ComputeBudget&) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    run_count_.fetch_add(1, std::memory_order_relaxed);
    return beluga_estimation::Err(beluga_estimation::Error::kFailure);
  }

  int GetRunCount() const { return run_count_.load(std::memory_order_relaxed); }

 private:
  std::shared_ptr<beluga_estimation::MiddlewareInterface> middleware_;
  std::string prefix_;
  std::atomic<int> run_count_{0};
};

}  // namespace beluga_estimation_ros::test

BELUGA_ESTIMATION_ROS_REGISTER_TYPE(beluga_estimation_ros::test::MockMethodQuick, beluga_estimation::BayesianProgram);
BELUGA_ESTIMATION_ROS_REGISTER_TYPE(beluga_estimation_ros::test::MockMethodSlow, beluga_estimation::BayesianProgram);
BELUGA_ESTIMATION_ROS_REGISTER_TYPE(beluga_estimation_ros::test::MockMethodFailing, beluga_estimation::BayesianProgram);
