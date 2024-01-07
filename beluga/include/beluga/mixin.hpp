// Copyright 2023-2024 Ekumen, Inc.
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

#include <memory>

#include <beluga/mixin/descriptor.hpp>
#include <beluga/mixin/particle_filter.hpp>
#include <beluga/mixin/sampling.hpp>
#include <beluga/mixin/storage.hpp>
#include <beluga/mixin/utility.hpp>
#include <ciabatta/ciabatta.hpp>

/**
 * \file
 * \brief Implementation of mixin utilities and extensions.
 */

namespace beluga::mixin {

/// Composes multiple interfaces into a single interface type.
/**
 * \tparam Interfaces Interfaces to combine.
 */
template <class... Interfaces>
struct compose_interfaces : public Interfaces... {
  /// Virtual destructor.
  /**
   * Makes it so at least one of the interfaces provides a virtual destructor.
   */
  ~compose_interfaces() override = default;
};

/// Helper method to get descriptor parameters or forward the value.
/**
 * \tparam T The type of the input value.
 * \param value A value to get the params from, or to forward.
 * \return The params with the same value category of the input value or
 * the input value forwarded.
 */
template <class T>
constexpr auto&& params_or_forward(T&& value) noexcept {
  if constexpr (is_descriptor_v<T> && has_params_v<T>) {
    return forward_like<T>(value.params);
  } else {
    return std::forward<T>(value);
  }
}

/// Constructs a mixin and wraps it into a `std::unique_ptr` to a given interface.
/**
 * This method can be used to create different mixin alternatives based on
 * the input arguments. The input arguments can be parameters of the mixin constructor
 * or descriptor variants holding parameters of the mixin constructor.
 *
 * This function will use the descriptor information to instantiate the correct
 * mixin type and construct it with the given parameters.
 *
 * Descriptors can also contain no parameters, in which case the type information
 * will be used to deduce the mixin type and there is no need to forward parameter
 * values to the constructor.
 *
 * All the possible mixin alternatives from the input variants must implement `Interface`,
 * and the `Interface` type must be specified since this function must return the
 * same type for all possible combinations of variants.
 *
 * Examples:
 * \snippet test/beluga/test_mixin.cpp Using make_mixin
 *
 * \tparam Interface The interface type used to create the std::unique_ptr.
 * \tparam Base A base mixin template taking descriptors.
 * \tparam Args The input argument or descriptor types from which to instantiate the mixin.
 * \param args The input arguments or descriptor instances from which to construct the mixin.
 * \return A `std::unique_ptr` of an instance of type `Interface`.
 */
template <class Interface, template <class...> class Base, class... Args>
auto make_mixin(Args&&... args) {
  return visit_everything(
      [](auto&&... args) {
        using InnerArgs = list<decltype(args)...>;  // avoid https://gcc.gnu.org/bugzilla/show_bug.cgi?id=86859
        using Concrete = mixin_from_descriptors_t<Base, filter<is_descriptor, InnerArgs>>;
        return std::apply(
            [](auto&&... args) -> std::unique_ptr<Interface> { return std::make_unique<Concrete>(args...); },
            make_tuple_with<is_not_descriptor>(params_or_forward(args)...));
      },
      args...);
}

}  // namespace beluga::mixin

#endif
