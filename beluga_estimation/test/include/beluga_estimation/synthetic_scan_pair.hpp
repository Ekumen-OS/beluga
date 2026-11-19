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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <stdexcept>
#include <vector>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/random.hpp>

namespace beluga_estimation::testing {

enum class SyntheticScene { kAsymmetricRoom, kCorridor, kRepeatedGeometry, kSparseNonCollinear };

template <typename T>
struct SyntheticScanCorruption_ {
  T retention_probability{T{1}};
  /// Standard deviation of Gaussian range noise, resampled until the range is positive.
  T noise_standard_deviation{T{0}};
  /// Number of retained beams replaced by a uniform range in [0, true_range).
  std::size_t short_return_count{0};
  /// Number of other retained beams replaced by a uniform range in [0, random_return_max_range).
  std::size_t random_return_count{0};
  T random_return_max_range{T{10}};
};

template <typename T>
struct SyntheticScanPair_ {
  std::vector<Vector2<T>> reference;
  std::vector<Vector2<T>> target;
  SE2<T> reference_T_target;
};

using SyntheticScanCorruption = SyntheticScanCorruption_<double>;
using SyntheticScanPair = SyntheticScanPair_<double>;

template <typename T>
std::vector<Vector2<T>> MakeSyntheticScene(SyntheticScene scene) {
  switch (scene) {
    case SyntheticScene::kAsymmetricRoom:
      return {{T{-4}, T{-3}}, {T{-1}, T{-3}}, {T{3}, T{-3}}, {T{3}, T{0}},
              {T{3}, T{2}},   {T{1}, T{2}},   {T{-4}, T{1}}, {T{-4}, T{-1}}};
    case SyntheticScene::kCorridor:
      return {{T{-5}, T{-2}}, {T{-2}, T{-2}}, {T{1}, T{-2}}, {T{5}, T{-2}},
              {T{-5}, T{2}},  {T{-1}, T{2}},  {T{2}, T{2}},  {T{5}, T{2}}};
    case SyntheticScene::kRepeatedGeometry:
      return {{T{-4}, T{-1}}, {T{-4}, T{1}}, {T{-2}, T{-1}}, {T{-2}, T{1}},
              {T{0}, T{-1}},  {T{0}, T{1}},  {T{2}, T{-1}},  {T{2}, T{1}}};
    case SyntheticScene::kSparseNonCollinear:
      return {{T{-2}, T{-1}}, {T{3}, T{0}}, {T{0}, T{4}}};
  }
  throw std::invalid_argument("Unknown synthetic scene");
}

/// Generate a target scan with range-only corruption in the target sensor frame.
/// Replacement beams are selected without replacement. Counts must fit the retained
/// nonzero-range points or std::invalid_argument is thrown. Zero-range points are kept
/// unchanged and excluded from corruption because their beam direction is undefined.
/// Gaussian noise applies only to beams not selected for replacement. Zero noise
/// preserves points exactly; positive noise is conditioned on a positive range.
template <typename T>
SyntheticScanPair_<T> MakeSyntheticScanPair(
    SyntheticScene scene,
    const SE2<T>& reference_T_target,
    RandomNumberGenerator& generator,
    const SyntheticScanCorruption_<T>& corruption = {}) {
  if (!reference_T_target.matrix().allFinite()) {
    throw std::invalid_argument("Transform must be finite");
  }
  if (!std::isfinite(corruption.retention_probability) || corruption.retention_probability < T{0} ||
      corruption.retention_probability > T{1}) {
    throw std::invalid_argument("Retention probability must be in [0, 1]");
  }
  if (!std::isfinite(corruption.noise_standard_deviation) || corruption.noise_standard_deviation < T{0}) {
    throw std::invalid_argument("Noise standard deviation must be finite and non-negative");
  }
  if (!std::isfinite(corruption.random_return_max_range) || corruption.random_return_max_range <= T{0}) {
    throw std::invalid_argument("Random return maximum range must be finite and positive");
  }

  SyntheticScanPair_<T> scan_pair{MakeSyntheticScene<T>(scene), {}, reference_T_target};
  std::bernoulli_distribution retain(static_cast<double>(corruption.retention_probability));
  const auto target_T_reference = reference_T_target.inverse();
  scan_pair.target.reserve(scan_pair.reference.size());
  std::vector<std::size_t> eligible;
  for (const auto& point : scan_pair.reference) {
    if (retain(generator)) {
      scan_pair.target.push_back(target_T_reference * point);
      if (scan_pair.target.back().stableNorm() > T{0}) {
        eligible.push_back(scan_pair.target.size() - 1);
      }
    }
  }
  if (corruption.short_return_count > eligible.size() ||
      corruption.random_return_count > eligible.size() - corruption.short_return_count) {
    throw std::invalid_argument("Replacement counts exceed the retained nonzero-range point count");
  }
  if (corruption.short_return_count > 0 || corruption.random_return_count > 0) {
    std::shuffle(eligible.begin(), eligible.end(), generator);
  }
  const auto short_count = corruption.short_return_count;
  const auto random_count = corruption.random_return_count;
  // A normal distribution requires a strictly positive standard deviation even
  // when no samples will be drawn.
  std::normal_distribution<T> noise(
      T{0}, corruption.noise_standard_deviation > T{0} ? corruption.noise_standard_deviation : T{1});
  for (std::size_t index = 0; index < eligible.size(); ++index) {
    auto& point = scan_pair.target[eligible[index]];
    const T range = point.stableNorm();
    T measured_range = range;
    if (index < short_count) {
      measured_range = std::uniform_real_distribution<T>(T{0}, range)(generator);
    } else if (index < short_count + random_count) {
      measured_range = std::uniform_real_distribution<T>(T{0}, corruption.random_return_max_range)(generator);
    } else if (corruption.noise_standard_deviation > T{0}) {
      do {
        measured_range = range + noise(generator);
      } while (measured_range <= T{0} || !std::isfinite(measured_range));
    } else {
      continue;
    }
    point = (point / range) * measured_range;
  }
  return scan_pair;
}

}  // namespace beluga_estimation::testing
