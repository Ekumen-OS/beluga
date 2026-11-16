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
 * @brief Defines Beluga's unified error type.
 *
 * `beluga_estimation::Error` is the error half of every `Result_<T>` and `Result` value. It
 * carries a machine-readable @ref Code and an optional human-readable
 * description. Errors form a chain: an error raised while an earlier one was
 * still in hand may carry that earlier error as its @ref GetCause "cause". The
 * head of the chain is the newest error, which is the one control flow reads;
 * the rest is diagnostic.
 *
 * An error may also carry @ref GetSideEffects "side effects": failures that
 * happened while the error was being handled (for example, cleanup failures).
 * Side effects are separate chains, not part of the causal chain, and each one
 * may itself carry a cause chain and further side effects.
 *
 * An `Error` is immutable after construction: its cause and side effects can
 * only be set by a constructor. To extend an error (for example, to attach a
 * cause while chaining), move-construct a new one from it with the extra
 * arguments.
 *
 * Only codes with control-flow consumers are defined. Diagnostics belong in the
 * description string, not in the code.
 */

#include <iterator>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <fmt/format.h>

namespace beluga_estimation {

/**
 * @brief Describes why an operation could not complete.
 *
 * The head of an error chain carries the code that control flow should act on.
 * Errors compare by code and description only; the cause chain and side effects
 * are diagnostic and are never part of equality.
 */
class Error {
 public:
  /// @brief Enumerates the error conditions Beluga's control flow distinguishes.
  enum Code : int {
    kBusy = 0,         ///< A required resource was unavailable; retry later.
    kInterrupted = 1,  ///< The operation was cancelled via a stop request.
    kTimeout = 2,      ///< The allotted time was exceeded before completion.
    kFailure = 3,      ///< Any other terminal failure; see the description.
  };

  /// @brief Constructs a failure with an empty description and no cause.
  Error() : code_(kFailure) {}

  /// @brief Constructs an error from a bare code (empty description, no cause).
  /// @param code The error code.
  // NOLINTNEXTLINE(google-explicit-constructor)
  Error(Code code) : code_(code) {}

  /// @brief Constructs a failure from a description.
  /// @param description Human-readable detail string.
  // NOLINTNEXTLINE(google-explicit-constructor)
  Error(std::string_view description) : code_(kFailure), description_(description) {}

  /// @brief Constructs an error from a code and a description.
  /// @param code The error code.
  /// @param description Human-readable detail string.
  Error(Code code, std::string_view description) : code_(code), description_(description) {}

  /// @brief Constructs an error from a code and a cause.
  /// @param code The error code.
  /// @param cause The earlier error this one interrupted.
  Error(Code code, Error&& cause) : code_(code), cause_(std::make_unique<Error>(std::move(cause))) {}

  /// @brief Constructs a failure from a description and a cause.
  /// @param description Human-readable detail string.
  /// @param cause The earlier error this one interrupted.
  Error(std::string_view description, Error&& cause)
      : code_(kFailure), description_(description), cause_(std::make_unique<Error>(std::move(cause))) {}

  /// @brief Constructs an error from a code, a description, and a cause.
  /// @param code The error code.
  /// @param description Human-readable detail string.
  /// @param cause The earlier error this one interrupted.
  Error(Code code, std::string_view description, Error&& cause)
      : code_(code), description_(description), cause_(std::make_unique<Error>(std::move(cause))) {}

  /// @brief Constructs an error from a code, a description, and side effects.
  /// @param code The error code.
  /// @param description Human-readable detail string.
  /// @param side_effects Failures to record as side effects.
  Error(Code code, std::string_view description, std::vector<Error>&& side_effects)
      : code_(code), description_(description), side_effects_(std::move(side_effects)) {}

  /// @brief Move-constructs an error from @p error, attaching @p cause.
  ///
  /// Any cause @p error already carried is replaced; its description and side
  /// effects are kept.
  ///
  /// @param error The error to adopt.
  /// @param cause The earlier error @p error interrupted.
  Error(Error&& error, Error&& cause)
      : code_(error.code_),
        description_(std::move(error.description_)),
        cause_(std::make_unique<Error>(std::move(cause))),
        side_effects_(std::move(error.side_effects_)) {}

  /// @brief Move-constructs an error from @p error, appending @p side_effects.
  ///
  /// @p error keeps its code, description, and cause chain; @p side_effects are
  /// appended after any it already carried.
  ///
  /// @param error The error to adopt.
  /// @param side_effects Failures to record as side effects.
  Error(Error&& error, std::vector<Error>&& side_effects)
      : code_(error.code_),
        description_(std::move(error.description_)),
        cause_(std::move(error.cause_)),
        side_effects_(std::move(error.side_effects_)) {
    side_effects_.insert(
        side_effects_.end(), std::make_move_iterator(side_effects.begin()),
        std::make_move_iterator(side_effects.end()));
  }

  Error(const Error&) = delete;
  Error& operator=(const Error&) = delete;
  Error(Error&& other) noexcept = default;
  Error& operator=(Error&& other) noexcept = default;
  ~Error() = default;

  /// @brief Returns the error code that control flow should act on.
  [[nodiscard]] Code GetCode() const noexcept { return code_; }

  /// @brief Returns the human-readable detail string, if any.
  [[nodiscard]] const std::string& GetDescription() const noexcept { return description_; }

  /// @brief Returns the earlier error this one interrupted, if any.
  ///
  /// Ownership stays with this error; the returned pointer is valid only as
  /// long as this error is.
  [[nodiscard]] const Error* GetCause() const noexcept { return cause_.get(); }

  /// @brief Returns the failures that happened while this one was handled.
  [[nodiscard]] const std::vector<Error>& GetSideEffects() const noexcept { return side_effects_; }

  /// @brief Compares two errors by code and description (never by chain or effects).
  friend bool operator==(const Error& lhs, const Error& rhs) noexcept {
    return lhs.code_ == rhs.code_ && lhs.description_ == rhs.description_;
  }

  /// @brief Compares two errors by code and description.
  friend bool operator!=(const Error& lhs, const Error& rhs) noexcept { return !(lhs == rhs); }

  /// @brief Compares an error's code with a bare code.
  friend bool operator==(const Error& error, Code code) noexcept { return error.code_ == code; }

  /// @brief Compares a bare code with an error's code.
  friend bool operator==(Code code, const Error& error) noexcept { return error.code_ == code; }

  /// @brief Compares an error's code with a bare code.
  friend bool operator!=(const Error& error, Code code) noexcept { return error.code_ != code; }

  /// @brief Compares a bare code with an error's code.
  friend bool operator!=(Code code, const Error& error) noexcept { return error.code_ != code; }

  /// @brief Writes this error, its cause chain, and its side effects to a stream.
  friend std::ostream& operator<<(std::ostream& os, const Error& error);

 private:
  Code code_;                          ///< The error code.
  std::string description_{};          ///< Optional human-readable detail string.
  std::unique_ptr<Error> cause_{};     ///< Optional earlier error.
  std::vector<Error> side_effects_{};  ///< Failures that happened while handling this one.
};

inline std::ostream& operator<<(std::ostream& os, const Error& error) {
  switch (error.code_) {
    case Error::kBusy:
      os << "BUSY";
      break;
    case Error::kInterrupted:
      os << "INTERRUPTED";
      break;
    case Error::kTimeout:
      os << "TIMEOUT";
      break;
    case Error::kFailure:
      os << "FAILURE";
      break;
    default:
      os << "UNKNOWN(" << static_cast<int>(error.code_) << ")";
      break;
  }
  if (!error.description_.empty()) {
    os << ": " << error.description_;
  }
  if (error.cause_ != nullptr) {
    os << " <- " << *error.cause_;
  }
  for (const auto& effect : error.side_effects_) {
    os << " [also: " << effect << "]";
  }
  return os;
}

}  // namespace beluga_estimation

/// @brief Formats a `beluga_estimation::Error`, including its cause chain and side effects.
template <>
struct fmt::formatter<beluga_estimation::Error> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const beluga_estimation::Error& error, FormatContext& context) const {
    std::ostringstream stream;
    stream << error;
    return fmt::formatter<std::string_view>::format(stream.str(), context);
  }
};
