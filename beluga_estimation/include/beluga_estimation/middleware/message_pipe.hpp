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

/**
 * @file
 * @brief Defines a generic queued subscription.
 */

#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/middleware/subscriber.hpp>

namespace beluga_estimation {

namespace detail {

/**
 * @brief Type-erased message queue backing @ref beluga_estimation::MessagePipe.
 *
 * The concrete implementation is hidden in a translation unit and owns a
 * lock-free queue of type-erased messages (`std::shared_ptr<void>`). The
 * owning `MessagePipe<MsgT>` template casts messages to and from
 * `std::shared_ptr<const MsgT>` with `std::static_pointer_cast`.
 *
 * @note Not intended to be used outside of @ref beluga_estimation::MessagePipe.
 */
class MessagePipeImpl {
 public:
  virtual ~MessagePipeImpl() = default;

  /// @brief Returns true if there are no queued messages.
  [[nodiscard]] virtual bool Empty() const = 0;

  /// @brief Enqueues a type-erased message.
  ///
  /// @param message The message to enqueue, as a type-erased pointer.
  virtual void Enqueue(std::shared_ptr<void> message) = 0;

  /// @brief Dequeues a single type-erased message, if any is available.
  ///
  /// @return The dequeued message, or nullptr if the queue is empty.
  [[nodiscard]] virtual std::shared_ptr<void> Dequeue() = 0;

  /// @brief Discards all queued messages.
  virtual void DiscardAll() = 0;
};

/// @brief Creates a new @ref MessagePipeImpl instance.
///
/// @return A shared pointer to the created implementation.
std::shared_ptr<MessagePipeImpl> CreateMessagePipeImpl();

}  // namespace detail

/**
 * @brief A queued subscription that buffers incoming messages for later consumption.
 *
 * A `MessagePipe` wraps a middleware subscription with a lock-free queue.
 * Subscriber callbacks (running on the middleware thread) enqueue messages,
 * while consumers (running on the compute thread) dequeue them with
 * `Fetch(Message*)` or `Fetch(Func&&)`.
 *
 * The pipe follows an explicit lifecycle:
 * - @ref Open() creates the subscription.
 * - @ref Close() destroys the subscription and discards queued messages.
 * - The destructor closes the pipe automatically if still open.
 *
 * Both @ref Open() and @ref Close() are idempotent. This class is not copyable
 * nor movable; create it in place.
 *
 * @tparam MsgT Message type published on the topic.
 */
template <typename MsgT>
class MessagePipe {
 public:
  /// @brief Value type of the queued messages.
  using value_type = std::shared_ptr<const MsgT>;

  /**
   * @brief Constructs a message pipe for the given topic.
   *
   * The pipe is initially closed; call @ref Open() to subscribe.
   *
   * @param middleware The middleware interface used to create the subscription.
   * @param topic_name The name of the topic to subscribe to.
   */
  MessagePipe(std::shared_ptr<MiddlewareInterface> middleware, std::string topic_name)
      : middleware_(std::move(middleware)),
        topic_name_(std::move(topic_name)),
        impl_(detail::CreateMessagePipeImpl()) {}

  MessagePipe(const MessagePipe&) = delete;
  MessagePipe& operator=(const MessagePipe&) = delete;
  MessagePipe(MessagePipe&&) = delete;
  MessagePipe& operator=(MessagePipe&&) = delete;

  /// @brief Returns true if the pipe is open (i.e. subscribed).
  [[nodiscard]] bool IsOpen() const { return static_cast<bool>(subscription_); }

  /// @brief Returns true if there are no queued messages.
  ///
  /// Note that under concurrent producers this is only an approximation, as the
  /// underlying lock-free queue reports an approximate size.
  [[nodiscard]] bool Empty() const { return impl_->Empty(); }

  /**
   * @brief Opens the pipe, creating the subscription if not already open.
   *
   * Subsequent messages published on the topic are queued until consumed by
   * @ref Fetch. This method is idempotent: calling it on an already open pipe
   * has no effect.
   *
   * @return Success, or an error if the subscription could not be created.
   */
  Result Open() {
    if (subscription_) {
      return Ok();
    }
    auto subscription =
        middleware_->CreateSubscriber<MsgT>(topic_name_, [impl = impl_](std::shared_ptr<const MsgT> message) {
          impl->Enqueue(std::const_pointer_cast<MsgT>(std::move(message)));
        });
    if (!subscription) {
      return Err(CollectError(subscription));
    }
    subscription_ = std::move(*subscription);
    return Ok();
  }

  /**
   * @brief Closes the pipe, destroying the subscription and discarding queued messages.
   *
   * The subscription is destroyed first so that no further messages are queued,
   * and any messages queued before the close are discarded. This method is
   * idempotent: calling it on an already closed pipe has no effect. It assumes
   * the middleware guarantees that no subscriber callbacks are in flight after
   * the subscription is destroyed.
   */
  void Close() {
    subscription_.reset();
    impl_->DiscardAll();
  }

  /**
   * @brief Dequeues a single message, if any is available.
   *
   * @return The dequeued message, or nullptr if the queue is empty.
   */
  [[nodiscard]] value_type Fetch() {
    auto raw = impl_->Dequeue();
    if (!raw) {
      return nullptr;
    }
    return std::static_pointer_cast<const MsgT>(std::move(raw));
  }

  /**
   * @brief Dequeues all queued messages, invoking `func` on each of them.
   *
   * Messages are consumed destructively and in arrival order. If `func` throws,
   * the exception propagates out of this method; the message being processed is
   * lost, and the remaining queued messages are left untouched.
   *
   * @tparam Func Function type accepting `value_type` (e.g. a lambda taking
   * `std::shared_ptr<const MsgT>`).
   * @param func The function to invoke for each queued message.
   * @return The number of processed messages.
   */
  template <typename Func>
  std::size_t Fetch(Func&& func) {
    std::size_t count = 0;
    while (auto raw = impl_->Dequeue()) {
      func(std::static_pointer_cast<const MsgT>(std::move(raw)));
      ++count;
    }
    return count;
  }

 private:
  std::shared_ptr<MiddlewareInterface> middleware_;
  std::string topic_name_;
  std::shared_ptr<detail::MessagePipeImpl> impl_;
  std::shared_ptr<Subscriber<MsgT>> subscription_;
};

}  // namespace beluga_estimation
