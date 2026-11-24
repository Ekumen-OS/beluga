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
 * @brief Provides `Result`, `Result_`, `Ok()`, `Err()`, and error collectors.
 *
 * These aliases wrap `tl::expected<T, beluga_estimation::Error>` to give operations that
 * can fail a return type that propagates errors explicitly instead of throwing.
 *
 * `Error` is move-only, so `Result` and `Result_<T>` are move-only too. Idioms:
 *
 * @code
 *   // Pure propagation — prefer returning the result directly.
 *   return result;
 *
 *   // Relocation — adopt an error into a different Result type, verbatim.
 *   return Err(CollectError(result));
 *
 *   // Value extraction — move the value out of a successful result.
 *   if (!result) return Err(CollectError(result));
 *   auto value = Unwrap(result);
 *
 *   // New error, optionally chaining a cause and recording side effects.
 *   return Err(Error::kFailure, "something broke");
 *   return Err(Error::kFailure, fmt::format("type '{}' is not registered", name));
 *   return Err(CollectError(second), CollectError(first));  // second's head, first as cause
 *   return Err(CollectError(outcome), CollectErrors(cleanup));  // relocation + side effects
 * @endcode
 */

#include <memory>
#include <sstream>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <tl/expected.hpp>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/error.hpp>

namespace beluga_estimation {

/// @brief Alias for `tl::expected<T, Error>` — a value-or-error result type.
template <typename T>
using Result_ = tl::expected<T, Error>;

/// @brief Alias for `tl::expected<void, Error>` — a success-or-error result type.
using Result = tl::expected<void, Error>;

/// @brief Returns a successful `Result`.
inline Result Ok() noexcept {
  return {};
}

/// @brief Wraps arguments into a `tl::unexpected<Error>` for returning errors.
///
/// Arguments are forwarded to an `Error` constructor. The call forms are:
/// - `Err()` — a bare `kFailure`.
/// - `Err(code)` — a failure with the given code.
/// - `Err(message)` — a `kFailure` with a description.
/// - `Err(code, message)` — a failure with code and description.
/// - `Err(code, cause)`, `Err(message, cause)`, `Err(code, message, cause)` —
///   a new head with the given cause attached.
/// - `Err(Error)` — relocate an existing error verbatim.
/// - `Err(Error, Error)` — adopt an error, attaching a prior error as its cause.
/// - `Err(Error, std::vector<Error>)` — adopt an error, appending side effects.
///
/// @param args Arguments forwarded to the `Error` constructor.
/// @return The error wrapped in `tl::unexpected`.
template <typename... Args>
tl::unexpected<Error> Err(Args&&... args) {
  return tl::make_unexpected(Error(std::forward<Args>(args)...));
}

/// @brief Moves the error out of a failed result.
///
/// The result is consumed: its error is moved out, leaving the result in a
/// moved-from state. The caller must have checked for failure first
/// (`if (!result) ...`); calling this on a successful result throws.
///
/// @param result The failed result to collect from.
/// @return The error, moved out.
template <typename T>
Error CollectError(tl::expected<T, Error>& result) {
  BELUGA_ESTIMATION_DEMAND(!result.has_value(), "cannot collect an error from a successful result");
  return std::move(result.error());
}

/// @brief Moves the error out of a failed result.
/// @param result The failed result to collect from.
/// @return The error, moved out.
template <typename T>
Error CollectError(tl::expected<T, Error>&& result) {
  return CollectError(result);
}

/// @brief Moves the errors out of failed results, in order.
///
/// Each result is consumed. Successful results contribute nothing.
///
/// @param results The results to collect from.
/// @return The collected errors, in argument order.
template <typename... Results>
std::vector<Error> CollectErrors(Results&&... results) {
  std::vector<Error> errors;
  auto collect = [&errors](auto&& result) {
    if (!result.has_value()) {
      errors.push_back(CollectError(std::forward<decltype(result)>(result)));
    }
  };
  (collect(std::forward<Results>(results)), ...);
  return errors;
}

/// @brief Moves the value out of a successful result.
///
/// The result is consumed. The caller must have checked for success first
/// (`if (!result) ...`); calling this on a failed result throws
/// `tl::bad_expected_access<Error>`.
///
/// @param result The result to unwrap.
/// @return The contained value, moved out.
template <typename T>
[[nodiscard]] T&& Unwrap(tl::expected<T, Error>& result) {
  return std::move(result).value();
}

/// @brief Moves the value out of a successful result.
/// @param result The result to unwrap.
/// @return The contained value, moved out.
template <typename T>
[[nodiscard]] T&& Unwrap(tl::expected<T, Error>&& result) {
  return std::move(result).value();
}

}  // namespace beluga_estimation
