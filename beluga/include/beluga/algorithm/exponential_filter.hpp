// Copyright 2022-2023 Ekumen, Inc.
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

#ifndef BELUGA_ALGORITHM_EXPONENTIAL_FILTER_HPP
#define BELUGA_ALGORITHM_EXPONENTIAL_FILTER_HPP

/**
 * \file
 * \brief Implementation of an exponential filter.
 */

namespace beluga {

/// Callable type implementing an exponential filter.
class ExponentialFilter {
 public:
  /// Constructs an exponential filter.
  /**
   * \param alpha The exponential filter smoothing factor.
   */
  explicit ExponentialFilter(double alpha) : alpha_{alpha} {}

  /// Resets the output of the exponential filter to zero.
  void reset() { output_ = 0.; }

  /// Updates the exponential filter output given an input.
  /**
   * \param input Next value to be exponentially filtered.
   */
  [[nodiscard]] double operator()(double input) {
    output_ += (output_ == 0.) ? input : alpha_ * (input - output_);
    return output_;
  }

 private:
  double output_{0.};
  double alpha_{0.};
};

}  // namespace beluga

#endif
