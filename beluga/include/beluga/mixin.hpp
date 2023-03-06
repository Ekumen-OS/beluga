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

#ifndef BELUGA_MIXIN_HPP
#define BELUGA_MIXIN_HPP

#include <beluga/mixin/descriptor.hpp>
#include <beluga/mixin/utility.hpp>
#include <ciabatta/ciabatta.hpp>

namespace beluga::mixin {

template <class T>
constexpr auto&& params_or_forward(T&& value) noexcept {
  if constexpr (is_descriptor_v<T>) {
    return forward_like<T>(value.params);
  } else {
    return std::forward<T>(value);
  }
}

template <class Interface, template <template <class> class...> class Base, class... Args>
auto make_unique(Args&&... args) {
  return visit_everything(
      [](auto&&... args) {
        using Concrete = mixin_from_descriptors_t<Base, filter<is_descriptor_or_tag, decltype(args)...>>;
        return std::apply(
            [](auto&&... args) -> std::unique_ptr<Interface> { return std::make_unique<Concrete>(args...); },
            make_tuple_with<is_not_tag>(params_or_forward(args)...));
      },
      args...);
}

}  // namespace beluga::mixin

#endif
