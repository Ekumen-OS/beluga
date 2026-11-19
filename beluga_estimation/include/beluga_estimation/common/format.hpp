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
 * @brief Defines lazy `fmt` formatters for Eigen data used in diagnostics.
 *
 * Logging a `beluga_estimation::VectorX` or a covariance matrix is common in operation
 * diagnostics, but building those strings eagerly costs an allocation per
 * logged value even when the logger discards the message. The views defined
 * here defer all rendering to `fmt`, which only runs when the target logger
 * emits the message (see `beluga_estimation::MiddlewareInterface::Debug` and
 * `beluga_estimation::LoggerInterface::IsEnabled`).
 *
 * Usage:
 * @code
 * middleware->Debug("predicted mean {} with covariance diag {}", VectorLike(mean), MatrixLike(covariance,
 * MatrixStyle::kDiagonal));
 * middleware->Debug("innovation matrix {}", MatrixLike(matrix));
 * @endcode
 *
 * Views hold a reference to the formatted expression, so they must not outlive
 * it. That is always the case when they are passed directly as log arguments.
 * Both accept arbitrary Eigen expressions, so blocks, `beluga_estimation::Ref` and
 * expression templates can be rendered without materializing them first.
 */

#include <algorithm>
#include <string>
#include <string_view>

#include <fmt/format.h>

#include <Eigen/Core>

namespace beluga_estimation {

/**
 * @brief Rendering style for matrix-like Eigen expressions.
 *
 * Used by @ref MatrixLike to choose between the diagonal, which is what
 * covariance and information matrices usually need in a log line, and the
 * complete matrix, which is the default.
 */
enum class MatrixStyle {
  /// Render only the diagonal, as `[m00, m11, ...]`.
  kDiagonal,
  /// Render every entry, as nested bracketed rows.
  kFull,
};

/**
 * @brief Rendering view for a vector-like Eigen expression, formatted as `[v0, v1, ...]`.
 *
 * Create instances with @ref VectorLike; the view is lightweight and holds a
 * reference to the source expression.
 *
 * @tparam Derived Eigen expression type being rendered.
 */
template <typename Derived>
struct VectorLikeView {
  /// Vector-like expression to render.
  const Eigen::MatrixBase<Derived>& source;
};

/**
 * @brief Creates a @ref VectorLikeView over a vector-like Eigen expression.
 *
 * Values are rendered with `fmt`'s `{:.6g}` presentation, which keeps log lines
 * short without losing the significant digits that matter for diagnostics.
 *
 * @tparam Derived Eigen expression type, statically known to be a vector.
 * @param values Vector-like expression to render; must outlive the returned view.
 * @return A lazy view suitable for use as an `fmt` format argument.
 */
template <typename Derived>
VectorLikeView<Derived> VectorLike(const Eigen::MatrixBase<Derived>& values) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  return VectorLikeView<Derived>{values};
}

/**
 * @brief Rendering view for a matrix-like Eigen expression.
 *
 * Create instances with @ref MatrixLike, choosing between diagonal-only and
 * full rendering. Full rendering is the default; covariance and information
 * matrices usually want `MatrixStyle::kDiagonal` instead, since it conveys
 * scale while a full matrix is unreadable in a log line.
 *
 * @tparam Derived Eigen expression type being rendered.
 */
template <typename Derived>
struct MatrixLikeView {
  /// Matrix-like expression to render.
  const Eigen::MatrixBase<Derived>& source;

  /// Rendering style to apply.
  MatrixStyle style;
};

/**
 * @brief Creates a @ref MatrixLikeView over a matrix-like Eigen expression.
 *
 * @tparam Derived Eigen expression type.
 * @param matrix Matrix-like expression to render; must outlive the returned view.
 * @param style Pass `MatrixStyle::kDiagonal` to render `[m00, m11, ...]`;
 *        defaults to `MatrixStyle::kFull`, which renders nested bracketed rows.
 * @return A lazy view suitable for use as an `fmt` format argument.
 */
template <typename Derived>
MatrixLikeView<Derived> MatrixLike(const Eigen::MatrixBase<Derived>& matrix, MatrixStyle style = MatrixStyle::kFull) {
  return MatrixLikeView<Derived>{matrix, style};
}

}  // namespace beluga_estimation

/// @brief Formats a @ref beluga_estimation::VectorLikeView as `[v0, v1, ...]`.
template <typename Derived>
struct fmt::formatter<beluga_estimation::VectorLikeView<Derived>> : fmt::formatter<std::string_view> {
  /**
   * @brief Renders the view into @p context.
   * @param view View to render.
   * @param context Destination format context.
   * @return Iterator past the last written character.
   */
  template <typename FormatContext>
  auto format(const beluga_estimation::VectorLikeView<Derived>& view, FormatContext& context) const {
    std::string rendered = "[";
    for (Eigen::Index i = 0; i < view.source.size(); ++i) {
      if (i > 0) {
        rendered += ", ";
      }
      rendered += fmt::format("{:.6g}", static_cast<double>(view.source(i)));
    }
    rendered += "]";
    return fmt::formatter<std::string_view>::format(rendered, context);
  }
};

/// @brief Formats a @ref beluga_estimation::MatrixLikeView as `[m00, m11, ...]` or `[[a, b], [c, d]]`.
template <typename Derived>
struct fmt::formatter<beluga_estimation::MatrixLikeView<Derived>> : fmt::formatter<std::string_view> {
  /**
   * @brief Renders the view into @p context.
   * @param view View to render.
   * @param context Destination format context.
   * @return Iterator past the last written character.
   */
  template <typename FormatContext>
  auto format(const beluga_estimation::MatrixLikeView<Derived>& view, FormatContext& context) const {
    std::string rendered = "[";
    if (view.style == beluga_estimation::MatrixStyle::kDiagonal) {
      const auto size = std::min(view.source.rows(), view.source.cols());
      for (Eigen::Index i = 0; i < size; ++i) {
        if (i > 0) {
          rendered += ", ";
        }
        rendered += fmt::format("{:.6g}", static_cast<double>(view.source(i, i)));
      }
    } else {
      for (Eigen::Index i = 0; i < view.source.rows(); ++i) {
        if (i > 0) {
          rendered += ", ";
        }
        rendered += "[";
        for (Eigen::Index j = 0; j < view.source.cols(); ++j) {
          if (j > 0) {
            rendered += ", ";
          }
          rendered += fmt::format("{:.6g}", static_cast<double>(view.source(i, j)));
        }
        rendered += "]";
      }
    }
    rendered += "]";
    return fmt::formatter<std::string_view>::format(rendered, context);
  }
};
