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
 * @brief Experimental scan-to-scan matching contract for planar point scans.
 */

#include <cstddef>
#include <optional>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>

namespace beluga_estimation::experimental {

/**
 * @brief A non-owning view of an unordered planar point scan.
 *
 * Coordinates are finite values in meters in a right-handed planar frame, with
 * x forward and y left. All points in both scans must already be expressed at a
 * common acquisition time.
 */
template <typename T>
using PlanarScanView = std_future::span<const Vector2<T>>;

/// @brief Outcome of a planar scan-matching attempt.
enum class PlanarScanMatchOutcome {
  kConverged,
  kRejected,
  kDegenerate,
  kNoConvergence,
  kInvalidInput,
  kNumericalFailure,
};

/** @brief Algorithm-specific diagnostics for a scan-matching attempt. */
template <typename T>
struct PlanarScanMatchDiagnostics_ {
  /// Score defined by the matcher; scores from different matchers are not comparable.
  std::optional<T> score;
  /// Number of algorithm iterations performed.
  std::size_t iteration_count{0};
  /// Number of transform candidates evaluated, when meaningful for the matcher.
  std::optional<std::size_t> candidate_count;
};

/** @brief Result of matching a target scan to a reference scan. */
template <typename T>
struct PlanarScanMatchResult_ {
  /// A converged result must contain an estimate.
  PlanarScanMatchOutcome outcome{PlanarScanMatchOutcome::kInvalidInput};
  /// Transform mapping target-scan points into the reference frame.
  std::optional<SE2<T>> reference_T_target;
  /// Additional matcher-specific information.
  PlanarScanMatchDiagnostics_<T> diagnostics;
};

/**
 * @brief Experimental interface for scan-to-scan planar matching.
 *
 * Input points are unordered. The initial guess and returned estimate use the
 * same `reference_T_target` convention and angles are in radians. A
 * `kConverged` result must contain an estimate. Rejected, degenerate, or
 * non-converged results may retain their best candidate. Invalid-input and
 * numerical-failure results must not contain an estimate.
 */
template <typename T>
class PlanarScanMatcher_ {
 public:
  virtual ~PlanarScanMatcher_() = default;

  /**
   * @brief Matches target points to reference points.
   * @param reference Reference-frame points.
   * @param target Target-frame points.
   * @param reference_T_target Initial transform mapping target into reference.
   * @return The matching outcome, optional estimate, and diagnostics.
   */
  virtual PlanarScanMatchResult_<T>
  Match(PlanarScanView<T> reference, PlanarScanView<T> target, const SE2<T>& reference_T_target) const = 0;
};

using PlanarScanMatchDiagnostics = PlanarScanMatchDiagnostics_<DefaultScalar>;
using PlanarScanMatchResult = PlanarScanMatchResult_<DefaultScalar>;
using PlanarScanMatcher = PlanarScanMatcher_<DefaultScalar>;

}  // namespace beluga_estimation::experimental
