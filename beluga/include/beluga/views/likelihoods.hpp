// Copyright 2025 Ekumen, Inc.
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

#ifndef BELUGA_VIEWS_LIKELIHOODS_HPP
#define BELUGA_VIEWS_LIKELIHOODS_HPP

#include <utility>

#include <beluga/views/particles.hpp>
#include <range/v3/view/transform.hpp>

/**
 * \file
 * \brief Implementation of the likelihoods range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// \brief Implementation detail for the likelihoods range adaptor object.
///
/// This struct follows the common C++ standard library pattern for creating
/// a customizable function object. We define a global constexpr instance of
/// this struct, which makes the view easy to use and compose.
struct likelihoods_fn {
  /// \brief Creates a view that computes the likelihood for each particle in a source range.
  /// \tparam Model A callable type that takes a particle's state and returns its likelihood (e.g., a double or float).
  /// \param model An instance of the sensor model callable.
  /// \return A range adaptor closure. When this closure is applied to a range of particles,
  ///         it returns a new lazy-evaluated view containing the likelihoods.
  template <class Model>
  constexpr auto operator()(Model model) const {
    // This is the core of the ranges composition pattern.
    // A view is created by chaining together other views using the pipe operator `|`.
    //
    // 1. `beluga::views::states`: This is the first adaptor in the chain. It takes a range
    //    of particles and produces a view of just their `state` members.
    //
    // 2. `ranges::views::transform(std::move(model))`: This is the second adaptor. It takes a
    //    range of elements (which will be the particle states from the previous step)
    //    and applies the `model` function to each one, producing a view of the results (the likelihoods).
    //
    // The result of piping these two adaptors together is a *new adaptor*. This new adaptor
    // is a "range adaptor closure" object that can be stored and later piped with an actual
    // range of particles to create the final view.
    return beluga::views::states | ranges::views::transform(std::move(model));
  }
};

}  // namespace detail

/// \brief A [range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject)
/// that produces a view of particle likelihoods.
///
/// This view takes a range of particles and a sensor model, and returns a new lazy-evaluated
/// range where each element is the likelihood of the corresponding particle's state, as
/// computed by the model.
///
/// ### Example
///
/// \code{.cpp}
/// auto particles = ...;
/// auto sensor_model = [](const auto& state) { return 0.5; };
///
/// // Create a lazy view of the likelihoods. No computation happens here.
/// auto likelihoods_view = particles | beluga::views::likelihoods(sensor_model);
///
/// // The model is called only as we iterate.
/// for (double likelihood : likelihoods_view) {
///   // use likelihood...
/// }
/// \endcode
inline constexpr detail::likelihoods_fn likelihoods;

}  // namespace beluga::views

#endif  // BELUGA_VIEWS_LIKELIHOODS_HPP
