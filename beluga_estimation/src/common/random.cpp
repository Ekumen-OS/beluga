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

#include <random>
#include <stdexcept>

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/random.hpp>

namespace beluga_estimation {

namespace {

template <typename ResultType>
struct default_prng;

template <>
struct default_prng<std::uint32_t> {
  using type = std::mt19937;
};

template <>
struct default_prng<std::uint64_t> {
  using type = std::mt19937_64;
};

template <typename ResultType>
using default_prng_t = typename default_prng<ResultType>::type;

}  // namespace

template <typename ResultType>
RandomNumberGenerator_<ResultType>::RandomNumberGenerator_() : RandomNumberGenerator_(std::random_device{}()) {}

template <typename ResultType>
RandomNumberGenerator_<ResultType>::RandomNumberGenerator_(result_type seed)
    : impl_(std::make_unique<Engine<default_prng_t<ResultType>>>(default_prng_t<ResultType>(seed))) {}

template <typename ResultType>
RandomNumberGenerator_<ResultType>::RandomNumberGenerator_(const RandomNumberGenerator_& other)
    : impl_(other.impl_ ? other.impl_->clone() : nullptr) {}

template <typename ResultType>
RandomNumberGenerator_<ResultType>& RandomNumberGenerator_<ResultType>::operator=(const RandomNumberGenerator_& other) {
  if (this != &other) {
    impl_ = other.impl_ ? other.impl_->clone() : nullptr;
  }
  return *this;
}

template <typename ResultType>
typename RandomNumberGenerator_<ResultType>::result_type RandomNumberGenerator_<ResultType>::operator()() {
  BELUGA_ESTIMATION_DEMAND(impl_ != nullptr, "RandomNumberGenerator has moved");
  return (*impl_)();
}

// Explicit instantiations
template class RandomNumberGenerator_<std::uint32_t>;
template class RandomNumberGenerator_<std::uint64_t>;

}  // namespace beluga_estimation
