// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_VIEWS_PARTICLES_HPP
#define BELUGA_VIEWS_PARTICLES_HPP

#include <beluga/primitives.hpp>
#include <range/v3/view/transform.hpp>

/**
 * \file
 * \brief Implementation of views related to particle ranges.
 */

namespace beluga::views {

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will obtain a reference to the state of each particle in the input range lazily.
inline constexpr auto states = ranges::views::transform(beluga::state);

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will obtain a reference to the weight of each particle in the input range lazily.
inline constexpr auto weights = ranges::views::transform(beluga::weight);

}  // namespace beluga::views

#endif
