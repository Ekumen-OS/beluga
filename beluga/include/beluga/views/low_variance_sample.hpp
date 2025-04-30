// Copyright 2023-2025 Ekumen, Inc.
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

#ifndef BELUGA_VIEWS_LOW_VARIANCE_SAMPLE_HPP
#define BELUGA_VIEWS_LOW_VARIANCE_SAMPLE_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#include <range/v3/all.hpp>

#include <beluga/type_traits/particle_traits.hpp>

namespace beluga {

namespace views {

namespace detail {

/// Implementation detail for a low_variance_sample range adaptor object.
struct low_variance_sample_fn {
  /// Overload that implements the low_variance_sample algorithm.
  /**
   * \tparam Range An [input range](https://en.cppreference.com/w/cpp/ranges/input_range) with particle states
   * or weights.
   * \param range Source range from where to sample elements.
   * \param n The number of samples to take (M in the algorithm).
   */
  template <class Range, std::enable_if_t<ranges::input_range<Range>, int> = 0>
  constexpr auto operator()(Range&& range, std::size_t n) const {
    static_assert(ranges::input_range<Range>);

    using RangeValue = ranges::range_value_t<Range>;
    std::vector<double> weights;
    std::vector<RangeValue> particles;  // Store particles for direct access

    if constexpr (is_particle_range_v<Range>) {
      weights.reserve(ranges::distance(range));
      particles.reserve(ranges::distance(range));
      ranges::for_each(range, [&](const RangeValue& particle) {
        weights.push_back(beluga::weight(particle));
        particles.push_back(particle);
      });
    } else {
      weights = ranges::to<std::vector<double>>(range);
      particles = ranges::to<std::vector<RangeValue>>(range);
    }

    const std::size_t num_particles = weights.size();
    if (num_particles == 0 || n == 0) {
      return ranges::empty(range);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distrib(0.0, 1.0);
    const double r = distrib(gen);
    const double inv_n = 1.0 / static_cast<double>(n);
    double c = weights[0];
    std::size_t i = 0;
    std::vector<RangeValue> resampled_particles;
    resampled_particles.reserve(n);

    for (std::size_t m = 1; m <= n; ++m) {
      const double u = r + static_cast<double>(m - 1) * inv_n;
      while (u > c) {
        i++;
        if (i < num_particles) {
          c += weights[i];
        } else {
          i = num_particles - 1;
          break;
        }
      }
      resampled_particles.push_back(particles[i]);
    }

    return resampled_particles | ranges::views::common;
  }

  /// Overload that returns a view closure to compose with other views.
  /**
   * \param n The number of samples to take.
   */
  constexpr auto operator()(std::size_t n) const {
    return ranges::make_view_closure(ranges::bind_back(low_variance_sample_fn{}, n));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will perform low-variance sampling on the input range using the provided algorithm.
/**
 * The input range should be a set of particles with associated weights.
 * This adaptor will return a new range of resampled particles using the low-variance sampling
 * algorithm described.
 */
inline constexpr detail::low_variance_sample_fn low_variance_sample;

}  // namespace views

}  // namespace beluga

#endif
