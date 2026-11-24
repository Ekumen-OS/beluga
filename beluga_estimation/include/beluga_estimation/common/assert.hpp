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
 * @brief Provides Beluga's assertion and demand macros.
 *
 * Beluga distinguishes two kinds of precondition checks:
 *
 * - @ref BELUGA_ESTIMATION_DEMAND — an always-armed check for conditions that callers can
 *   and must satisfy at cold boundaries (construction, configuration, public
 *   entry points) or for unsupported operations. On failure it throws
 *   `std::runtime_error` carrying the condition text, function, file, and line,
 *   plus an optional fmt-style detail message. It cannot be disabled.
 *
 * - @ref BELUGA_ESTIMATION_ASSERT — a debug-only check for invariants that hold by
 *   construction. By the time execution reaches a hot path, all caller-facing
 *   preconditions should already have been demanded, so hot-path checks assert
 *   instead: they abort in debug builds and compile out in release builds.
 *
 * Both macros syntax-check that the condition is convertible to `bool` and is
 * not a raw pointer. Write `BELUGA_ESTIMATION_DEMAND(foo != nullptr)`, never
 * `BELUGA_ESTIMATION_DEMAND(foo)`.
 *
 * The condition is evaluated exactly once, even when it is an expression with
 * side effects, so it is safe to pass a function call or other expensive
 * expression directly.
 *
 * When the condition is a `Result`/`Result_<T>`, the failure message includes
 * the bundled error (code, description, cause chain, and side effects):
 *
 * @code
 * BELUGA_ESTIMATION_DEMAND(ptr != nullptr);
 * BELUGA_ESTIMATION_DEMAND(x > 0, "expected positive value, got {}", x);
 * BELUGA_ESTIMATION_DEMAND(result);  // reports the error carried by `result`
 * BELUGA_ESTIMATION_ASSERT(state == State::kInitialized);
 * @endcode
 */

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <fmt/format.h>

#include <beluga_estimation/common/error.hpp>

// ---------------------------------------------------------------------------
// Compile-time detection of the armed/disarmed state.
// ---------------------------------------------------------------------------

#ifdef BELUGA_ESTIMATION_ASSERT_IS_ARMED
#error Unexpected BELUGA_ESTIMATION_ASSERT_IS_ARMED defined.
#endif
#ifdef BELUGA_ESTIMATION_ASSERT_IS_DISARMED
#error Unexpected BELUGA_ESTIMATION_ASSERT_IS_DISARMED defined.
#endif

#if defined(BELUGA_ESTIMATION_ENABLE_ASSERTS) && defined(BELUGA_ESTIMATION_DISABLE_ASSERTS)
#error Conflicting assertion toggles: both BELUGA_ESTIMATION_ENABLE_ASSERTS and BELUGA_ESTIMATION_DISABLE_ASSERTS are defined.
#elif defined(BELUGA_ESTIMATION_ENABLE_ASSERTS) && defined(NDEBUG)
#error Conflicting assertion toggles: both BELUGA_ESTIMATION_ENABLE_ASSERTS and NDEBUG are defined
#elif defined(BELUGA_ESTIMATION_ENABLE_ASSERTS)
#define BELUGA_ESTIMATION_ASSERT_IS_ARMED
#elif defined(BELUGA_ESTIMATION_DISABLE_ASSERTS) || defined(NDEBUG)
#define BELUGA_ESTIMATION_ASSERT_IS_DISARMED
#else
#define BELUGA_ESTIMATION_ASSERT_IS_ARMED
#endif

namespace beluga_estimation {
namespace internal {

/// @brief Detection idiom: evaluates to `true` if `T` is a `Result`-like type.
template <typename T, typename = void>
struct is_result : std::false_type {};

template <typename T>
struct is_result<T, std::void_t<typename T::error_type>> : std::is_same<typename T::error_type, Error> {};

/// @brief Evaluates to `true` if `T` is a `Result`-like type.
template <typename T>
inline constexpr bool is_result_v = is_result<std::decay_t<T>>::value;

/// @brief Builds the shared prefix of an assertion/demand failure message.
/// @param kind Either `"BELUGA_ESTIMATION_DEMAND"` or `"BELUGA_ESTIMATION_ASSERT"`.
/// @param condition_expression The source text of the failed condition.
/// @param func Calling function name.
/// @param file Source file name.
/// @param line Source line number.
/// @return The message prefix.
inline std::string
FailurePrefix(const char* kind, const char* condition_expression, const char* func, const char* file, int line) {
  return std::string(kind) + "(" + condition_expression + ") failed at " + file + ":" + std::to_string(line) + " in " +
         func + "()";
}

/// @brief Appends a fmt-style detail message to @p message, if any.
/// @param message The message to append to.
/// @param format The format string.
/// @param args The format arguments.
template <typename... Args>
void AppendDetail(std::string& message, fmt::format_string<Args...> format, Args&&... args) {
  const std::string formatted = fmt::format(format, std::forward<Args>(args)...);
  if (!formatted.empty()) {
    message += ": ";
    message += formatted;
  }
}

/// @brief Appends the error carried by a failed result to @p message.
/// @param message The message to append to.
/// @param result The failed result.
template <typename Result>
void AppendResultError(std::string& message, const Result& result) {
  message += ": ";
  message += fmt::format("{}", result.error());
}

/// @brief Aborts the program with a formatted assertion-failure message.
/// @param message The complete failure message.
[[noreturn]] inline void AbortWithMessage(const std::string& message) {
  std::fprintf(stderr, "%s\n", message.c_str());
  std::abort();
}

/// @brief Throws `std::runtime_error` with a formatted demand-failure message.
/// @param message The complete failure message.
[[noreturn]] inline void ThrowWithMessage(const std::string& message) {
  throw std::runtime_error(message);
}

}  // namespace internal
}  // namespace beluga_estimation

// ---------------------------------------------------------------------------
// BELUGA_ESTIMATION_DEMAND — always armed, throws std::runtime_error on failure.
//
// The variadic entry point forwards to an implementation that always receives
// at least one variadic argument (a trailing empty format string). This keeps
// `BELUGA_ESTIMATION_DEMAND(condition)` valid even under `-Wpedantic`, which would
// otherwise reject a variadic macro invoked with no variadic arguments.
//
// The condition is bound to a local before being tested so that it is
// evaluated exactly once, even when it is an expression with side effects and
// even when the failure path reads it again to report a bundled error.
// ---------------------------------------------------------------------------

#define BELUGA_ESTIMATION_DETAIL_RAW_POINTER_STATIC_ASSERT(kind, condition)           \
  static_assert(                                                                      \
      !std::is_pointer_v<decltype(condition)>, "When using " kind                     \
                                               " on a raw pointer, always write out " \
                                               "\"" kind                              \
                                               "(foo != nullptr)\", do not write "    \
                                               "\"" kind "(foo)\" and rely on implicit pointer-to-bool conversion.")

#define BELUGA_ESTIMATION_DETAIL_BOOL_STATIC_ASSERT(condition) \
  static_assert(std::is_constructible_v<bool, decltype(condition)>, "Condition must be convertible to bool.")

#define BELUGA_ESTIMATION_DEMAND(...) BELUGA_ESTIMATION_DEMAND_IMPL(__VA_ARGS__, "")

#define BELUGA_ESTIMATION_DEMAND_IMPL(condition_expression, ...)                                          \
  do {                                                                                                    \
    auto&& beluga_detail_condition = condition_expression;                                                \
    BELUGA_ESTIMATION_DETAIL_RAW_POINTER_STATIC_ASSERT("BELUGA_ESTIMATION_DEMAND", condition_expression); \
    BELUGA_ESTIMATION_DETAIL_BOOL_STATIC_ASSERT(condition_expression);                                    \
    if (!static_cast<bool>(beluga_detail_condition)) {                                                    \
      std::string beluga_detail_message = ::beluga_estimation::internal::FailurePrefix(                   \
          "BELUGA_ESTIMATION_DEMAND", #condition_expression, __func__, __FILE__, __LINE__);               \
      if constexpr (::beluga_estimation::internal::is_result_v<decltype(beluga_detail_condition)>) {      \
        ::beluga_estimation::internal::AppendResultError(beluga_detail_message, beluga_detail_condition); \
      }                                                                                                   \
      ::beluga_estimation::internal::AppendDetail(beluga_detail_message, __VA_ARGS__);                    \
      ::beluga_estimation::internal::ThrowWithMessage(beluga_detail_message);                             \
    }                                                                                                     \
  } while (false)

// ---------------------------------------------------------------------------
// BELUGA_ESTIMATION_ASSERT — debug-only, aborts on failure when armed.
// ---------------------------------------------------------------------------

#ifdef BELUGA_ESTIMATION_ASSERT_IS_ARMED

namespace beluga_estimation {
/// @brief True when @ref BELUGA_ESTIMATION_ASSERT is armed in this translation unit.
constexpr bool kBelugaEstimationAssertIsArmed = true;
/// @brief True when @ref BELUGA_ESTIMATION_ASSERT is disarmed in this translation unit.
constexpr bool kBelugaEstimationAssertIsDisarmed = false;
}  // namespace beluga_estimation

#define BELUGA_ESTIMATION_ASSERT(...) BELUGA_ESTIMATION_ASSERT_IMPL(__VA_ARGS__, "")

#define BELUGA_ESTIMATION_ASSERT_IMPL(condition_expression, ...)                                          \
  do {                                                                                                    \
    auto&& beluga_detail_condition = condition_expression;                                                \
    BELUGA_ESTIMATION_DETAIL_RAW_POINTER_STATIC_ASSERT("BELUGA_ESTIMATION_ASSERT", condition_expression); \
    BELUGA_ESTIMATION_DETAIL_BOOL_STATIC_ASSERT(condition_expression);                                    \
    if (!static_cast<bool>(beluga_detail_condition)) {                                                    \
      std::string beluga_detail_message = ::beluga_estimation::internal::FailurePrefix(                   \
          "BELUGA_ESTIMATION_ASSERT", #condition_expression, __func__, __FILE__, __LINE__);               \
      if constexpr (::beluga_estimation::internal::is_result_v<decltype(beluga_detail_condition)>) {      \
        ::beluga_estimation::internal::AppendResultError(beluga_detail_message, beluga_detail_condition); \
      }                                                                                                   \
      ::beluga_estimation::internal::AppendDetail(beluga_detail_message, __VA_ARGS__);                    \
      ::beluga_estimation::internal::AbortWithMessage(beluga_detail_message);                             \
    }                                                                                                     \
  } while (false)

#else  // BELUGA_ESTIMATION_ASSERT_IS_DISARMED

namespace beluga_estimation {
/// @brief True when @ref BELUGA_ESTIMATION_ASSERT is armed in this translation unit.
constexpr bool kBelugaEstimationAssertIsArmed = false;
/// @brief True when @ref BELUGA_ESTIMATION_ASSERT is disarmed in this translation unit.
constexpr bool kBelugaEstimationAssertIsDisarmed = true;
}  // namespace beluga_estimation

#define BELUGA_ESTIMATION_ASSERT(...) BELUGA_ESTIMATION_ASSERT_IMPL(__VA_ARGS__, "")

#define BELUGA_ESTIMATION_ASSERT_IMPL(condition_expression, ...)                                          \
  do {                                                                                                    \
    BELUGA_ESTIMATION_DETAIL_RAW_POINTER_STATIC_ASSERT("BELUGA_ESTIMATION_ASSERT", condition_expression); \
    BELUGA_ESTIMATION_DETAIL_BOOL_STATIC_ASSERT(condition_expression);                                    \
  } while (false)

#endif
