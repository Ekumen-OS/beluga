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

#ifndef BELUGA_VIEWS_ELEMENTS_HPP
#define BELUGA_VIEWS_ELEMENTS_HPP

#include <tuple>

#include <range/v3/view/transform.hpp>

/**
 * \file
 * \brief Implementation of the C++20's std::views::elements view.
 */

namespace beluga::views {

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will apply `std::get<N>` to each value in the range lazily.
/**
 * \tparam N Element to get from the array or tuple.
 * \param tuple Tuple or array instance, with at least `N + 1` elements.
 */
template <std::size_t N>
inline constexpr auto elements = ranges::views::transform(
    [](auto&& tuple) -> decltype(auto) { return std::get<N>(std::forward<decltype(tuple)>(tuple)); });

}  // namespace beluga::views

#endif