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

#include <tuple>

#include <range/v3/view/transform.hpp>

namespace beluga::views {

template <std::size_t N>
inline auto elements = ranges::views::transform([](auto&& particle) -> decltype(auto) {
  return std::get<N>(std::forward<decltype(particle)>(particle));
});

}  // namespace beluga::views
