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

#ifndef BELUGA_VIEWS_TAKE_WHILE_KLD_HPP
#define BELUGA_VIEWS_TAKE_WHILE_KLD_HPP

#include <unordered_set>

#include <beluga/views/elements.hpp>

#include <range/v3/view/take.hpp>
#include <range/v3/view/take_while.hpp>
#include <range/v3/view/zip.hpp>

/**
 * \file
 * \brief Implementation of a take_while_kld range adaptor object.
 */

namespace beluga {

namespace detail {

/// Default upper standard normal quantile, P = 0.999
constexpr double kDefaultKldZ = 3.;

}  // namespace detail

/// Returns a callable object that verifies if the KLD condition is being satisfied.
/**
 * The callable object will compute the minimum number of samples based on a Kullback-Leibler
 * distance epsilon between the maximum likelihood estimate and the true distribution. \n
 * Z is the upper standard normal quantile for P, where P is the probability
 * that the error in the estimated distribution will be less than epsilon.
 *
 * Here are some examples:
 * | P     | Z                |
 * |-------|------------------|
 * | 0.900 | 1.28155156327703 |
 * | 0.950 | 1.64485362793663 |
 * | 0.990 | 2.32634787735669 |
 * | 0.999 | 3.09023224677087 |
 *
 * If the computed value is less than what the min argument specifies, then min will be returned.
 *
 * See KLD-Sampling: Adaptive Particle Filters \cite fox2001adaptivekldsampling.
 *
 * \param min Minimum number of particles that the callable object will return.
 * \param epsilon Maximum distance epsilon between the maximum likelihood estimate and the true
 *  distrubution.
 * \param z Upper standard normal quantile for the probability that the error in the
 *  estimated distribution is less than epsilon.
 * \return A callable object with prototype `(std::size_t hash) -> bool`.
 *  `hash` is the spatial hash of the particle being added. \n
 *  The returned callable object is stateful, tracking the total number of particles and
 *  the particle clusters based on the spatial hash. \n
 *  The return value of the callable will be false when the number of particles is more than the minimum
 *  and the KLD condition is satisfied, if not it will be true. \n
 *  i.e. A return value of true means that you need to keep sampling to satisfy the condition.
 */
inline auto kld_condition(std::size_t min, double epsilon, double z = beluga::detail::kDefaultKldZ) {
  auto target_size = [two_epsilon = 2 * epsilon, z](std::size_t k) {
    if (k <= 2U) {
      return std::numeric_limits<std::size_t>::max();
    }
    double common = 2. / static_cast<double>(9 * (k - 1));
    double base = 1. - common + std::sqrt(common) * z;
    double result = (static_cast<double>(k - 1) / two_epsilon) * base * base * base;
    return static_cast<std::size_t>(std::ceil(result));
  };

  return [=, count = 0ULL, buckets = std::unordered_set<std::size_t>{}](std::size_t hash) mutable {
    count++;
    buckets.insert(hash);
    return count <= min || count <= target_size(buckets.size());
  };
}

namespace views {

namespace detail {

/// Implementation detail for a take_while_kld range adaptor object.
struct take_while_kld_fn {
  /// Overload that implements the take_while_kld algorithm.
  /**
   * \tparam States An [input range](https://en.cppreference.com/w/cpp/ranges/input_range) with particle states.
   * \tparam Hashes An [input range](https://en.cppreference.com/w/cpp/ranges/input_range) with particle hashes.
   * \param states Source range from where to take elements.
   * \param hashes Source range containing hashes (or bucket ids) to compute the KLD condition.
   * \param min Minimum samples to take.
   * \param max Maximum samples to take.
   * \param epsilon See beluga::kld_condition() for details.
   * \param z See beluga::kld_condition() for details.
   */
  template <class States, class Hashes, std::enable_if_t<ranges::input_range<Hashes>, int> = 0>
  constexpr auto operator()(
      States&& states,
      Hashes&& hashes,
      std::size_t min,
      std::size_t max,
      double epsilon,
      double z = beluga::detail::kDefaultKldZ) const {
    static_assert(ranges::input_range<States>);
    static_assert(ranges::input_range<Hashes>);
    static_assert(std::is_convertible_v<ranges::range_value_t<Hashes>, std::size_t>);
    const auto hash = [](const auto& p) { return std::get<1>(p); };
    return ranges::views::zip(ranges::views::all(states), ranges::views::all(hashes)) |  //
           ranges::views::take_while(beluga::kld_condition(min, epsilon, z), hash) |     //
           ranges::views::take(max) |                                                    //
           beluga::views::elements<0>;
  }

  /// Overload that implements the take_while_kld algorithm with an external hasher.
  /**
   * \tparam States An [input range](https://en.cppreference.com/w/cpp/ranges/input_range) with particle states.
   * \tparam Hasher A callable object that can compute the spatial hash for a given state.
   * \param states Source range from where to take elements.
   * \param hasher Hasher instance used to compute the spatial hash for a given state.
   * \param min Minimum samples to take.
   * \param max Maximum samples to take.
   * \param epsilon See beluga::kld_condition() for details.
   * \param z See beluga::kld_condition() for details.
   */
  template <class States, class Hasher, std::enable_if_t<ranges::views::transformable_range<States, Hasher>, int> = 0>
  constexpr auto operator()(
      States&& states,
      Hasher hasher,
      std::size_t min,
      std::size_t max,
      double epsilon,
      double z = beluga::detail::kDefaultKldZ) const {
    return this->operator()(
        ranges::views::all(states),                           //
        ranges::views::transform(states, std::move(hasher)),  //
        min,                                                  //
        max,                                                  //
        epsilon,                                              //
        z);
  }

  /// Overload that returns a view closure to compose with other views.
  /**
   * \tparam Hasher A callable object that can compute the spatial hash for a given state.
   * \param hasher Hasher instance used to compute the spatial hash for a given state.
   * \param min Minimum samples to take.
   * \param max Maximum samples to take.
   * \param epsilon See beluga::kld_condition() for details.
   * \param z See beluga::kld_condition() for details.
   */
  template <class Hasher>
  constexpr auto operator()(
      Hasher hasher,
      std::size_t min,
      std::size_t max,
      double epsilon,
      double z = beluga::detail::kDefaultKldZ) const {
    return ranges::make_view_closure(ranges::bind_back(take_while_kld_fn{}, std::move(hasher), min, max, epsilon, z));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will take elements from a range while the KLD condition is statisfied.
/**
 * The input range should be randomly generated from an existing particle set.
 * This adaptor will return elements until the minimum number of samples based on a Kullback-Leibler
 * distance epsilon is satisfied.
 *
 * See KLD-Sampling: Adaptive Particle Filters \cite fox2001adaptivekldsampling.
 */
inline constexpr detail::take_while_kld_fn take_while_kld;

}  // namespace views

}  // namespace beluga

#endif
