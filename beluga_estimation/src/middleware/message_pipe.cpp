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
#include <utility>

#include <moodycamel/concurrentqueue.h>

#include <beluga_estimation/middleware/message_pipe.hpp>

namespace beluga_estimation {
namespace detail {

namespace {

/**
 * @brief Lock-free message queue implementation backing @ref beluga_estimation::detail::MessagePipeImpl.
 *
 * Owns a moodycamel concurrent queue of type-erased messages.
 */
class ConcurrentMessagePipeImpl final : public MessagePipeImpl {
 public:
  bool Empty() const override { return queue_.size_approx() == 0; }

  void Enqueue(std::shared_ptr<void> message) override { queue_.enqueue(std::move(message)); }

  std::shared_ptr<void> Dequeue() override {
    std::shared_ptr<void> message;
    queue_.try_dequeue(message);
    return message;
  }

  void DiscardAll() override {
    std::shared_ptr<void> message;
    while (queue_.try_dequeue(message)) {
    }
  }

 private:
  moodycamel::ConcurrentQueue<std::shared_ptr<void>> queue_;
};

}  // namespace

std::shared_ptr<MessagePipeImpl> CreateMessagePipeImpl() {
  return std::make_shared<ConcurrentMessagePipeImpl>();
}

}  // namespace detail
}  // namespace beluga_estimation
