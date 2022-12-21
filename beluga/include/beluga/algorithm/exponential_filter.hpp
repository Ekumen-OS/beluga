// Copyright 2022 Ekumen, Inc.
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

namespace beluga {

class ExponentialFilter {
 public:
  explicit ExponentialFilter(double alpha) : alpha_{alpha} {}

  void reset() { output_ = 0.; }

  [[nodiscard]] double operator()(double input) {
    output_ += (output_ == 0.) ? input : alpha_ * (input - output_);
    return output_;
  }

 private:
  double output_{0.};
  double alpha_{0.};
};

}  // namespace beluga
