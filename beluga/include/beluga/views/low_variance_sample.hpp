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

#ifndef BELUGA_VIEWS_LOW_VARIANCE_SAMPLE_HPP
#define BELUGA_VIEWS_LOW_VARIANCE_SAMPLE_HPP

#include <random>

#include <range/v3/utility/random.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/repeat.hpp>
#include <range/v3/view/take.hpp>

#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>

/**
 * \file
 * \brief Implementation of a sample (with replacement) range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// Implementation of the low variance sample view.
/// This algorithm is designed to sample elements from a range with replacement, ensuring that the probability of
/// selecting each element is proportional to its weight. It works with a single random number r in
/// the [0, 1/M) interval, where M is the total number of elements in the range. Then, starting from this random number
/// r, the algorithm repeatedly adds a fixed step size of 1/M to r. This step size ensures that the sampling process
/// progresses evenly across the range. After that, for each resulting value, the algorithm selects the element whose
/// cumulative weight corresponds to the current value of r. This ensures that elements with higher weights are more
/// likely to be selected. Finally, the process continues until the desired number of samples is obtained.
/// [Based on] Sebastian Thrun, Wolfram Burgard, and Dieter Fox. 2005. Probabilistic Robotics (Intelligent Robotics and
/// Autonomous Agents). The MIT Press.

/**
 * \tparam Range A [random access](https://en.cppreference.com/w/cpp/ranges/random_access_range) and
 *  [sized](https://en.cppreference.com/w/cpp/ranges/sized_range) range.
 * \tparam Weights
 * \tparam URNG A random number generator that satisfies the
 *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
 *  requirements.
 */
template <class Range, class Weights, class URNG = typename ranges::detail::default_random_engine>
struct low_variance_sample_view
    : public ranges::view_facade<low_variance_sample_view<Range, Weights, URNG>, ranges::infinite> {
 public:
  /// Default constructor.
  low_variance_sample_view() = default;

  /// Construct the view from an existing range.
  /**
   * \param range The range to be adapted.
   * \param weights The weights associated with the elements in the range.
   * \param engine The random number generator object.
   */
  constexpr low_variance_sample_view(Range range, Weights weights, URNG& engine = ranges::detail::get_random_engine())
      : range_{std::move(range)}, weights_{std::move(weights)}, engine_{std::addressof(engine)} {
    assert(ranges::size(range) > 0);
  }

 private:
  // `ranges::range_access` needs access to the cursor members.
  friend ranges::range_access;

  static_assert(ranges::sized_range<Range>);
  static_assert(ranges::random_access_range<Range>);
  static_assert(ranges::input_range<Weights>);

  /// Cursor class that handles the iteration logic.
  struct cursor {
   public:
    /// Default constructor.
    cursor() = default;

    /// Construct a cursor from the parent view elements.
    constexpr explicit cursor(low_variance_sample_view* view)
        : view_(view),
          range_begin_{ranges::begin(view_->range_)},  // Begin iterator of the range
          it_{range_begin_},
          weights_begin_{ranges::begin(view_->weights_)},          // Begin iterator of the weights
          M_{static_cast<uint64_t>(ranges::size(view_->range_))},  // Number of particles to be sampled
          r_{std::uniform_real_distribution<double>(0.0, 1.0 / static_cast<double>(M_))(
              *view_->engine_)},  // random number in [0, 1/M)
          m_{0},                  // Current sample index
          c_{*weights_begin_}     // Cumulative weight of the first particle
    {}

    /// Access the current iterator.
    [[nodiscard]] constexpr decltype(auto) read() const noexcept(noexcept(*this->it_)) { return *it_; }

    /// Position the current iterator.
    constexpr void next() {
      // A number U in [0, 1] that points to exactly one particle in the range.
      // Where the particle i satisfies with i=argmin_j \sum_1^j w^m >= U.
      const double U = r_ + static_cast<double>(m_) / static_cast<double>(M_);
      while (it_ != ranges::end(view_->range_) && U > c_) {
        ++it_;
        ++weights_begin_;
        c_ += *weights_begin_;
      }
      ++m_;
    }

   private:
    low_variance_sample_view* view_;
    ranges::iterator_t<Range> range_begin_;
    ranges::iterator_t<Range> it_;
    ranges::iterator_t<Weights> weights_begin_;
    uint64_t M_;
    double r_;
    uint64_t m_;
    ranges::range_value_t<Weights> c_;
  };

  /// Return the cursor for the begin iterator.
  [[nodiscard]] constexpr auto begin_cursor() { return cursor{this}; }

  /// Return an unreachable sentinel since this is an infinite range.
  [[nodiscard]] constexpr auto end_cursor() const noexcept { return ranges::unreachable_sentinel_t{}; }

  Range range_;
  Weights weights_;
  URNG* engine_;
};

/// Implementation detail for a low_variance_sample algorithm.
struct low_variance_sample_base_fn {
 protected:
  /// Sample from weighted ranges.
  template <class Range, class Weights, class URNG>
  constexpr auto low_variance_sample_from_range(Range&& range, Weights&& weights, URNG& engine) const {
    static_assert(ranges::sized_range<Range>);
    static_assert(ranges::random_access_range<Range>);
    static_assert(ranges::input_range<Weights>);
    return low_variance_sample_view{ranges::views::all(std::forward<Range>(range)), std::move(weights), engine};
  }

  /// Sample from any range.
  /**
   * If the input range is a particle range, it will extract the weights and treat it as a weighted range.
   * The new particles will all have a weight equal to 1, since, after resampling, the probability will be
   * represented by the number of particles rather than their individual weight.
   *
   * If the input range is not a particle range, it will assume a uniform distribution.
   */
  template <class Range, class URNG>
  constexpr auto low_variance_sample_from_range(Range&& range, URNG& engine) const {
    static_assert(ranges::sized_range<Range>);
    static_assert(ranges::random_access_range<Range>);
    if constexpr (beluga::is_particle_range_v<Range>) {
      return low_variance_sample_from_range(beluga::views::states(range), beluga::views::weights(range), engine) |
             ranges::views::transform(beluga::make_from_state<ranges::range_value_t<Range>>);
    } else {
      // Generate uniform weights if no weights are provided.
      auto uniform_weights = ranges::views::repeat(1.0) | ranges::views::take(ranges::size(range));
      return low_variance_sample_from_range(std::forward<Range>(range), uniform_weights, engine);
    }
  }
};

/// Implementation detail for a sample range adaptor object.
struct low_variance_sample_fn : public low_variance_sample_base_fn {
  /// Overload that takes three arguments.
  template <class Range, class Weights, class RandomEngine>
  constexpr auto operator()(Range&& range, Weights&& weights, RandomEngine& random_engine) const {
    static_assert(ranges::range<Range>);
    static_assert(ranges::range<Weights>);
    return low_variance_sample_from_range(std::forward<Range>(range), std::forward<Weights>(weights), random_engine);
  }

  /// Overload that takes two arguments.
  template <class Range, class Weights>
  constexpr auto operator()(Range&& range, Weights&& weights) const {
    if constexpr (ranges::range<Range> && ranges::range<Weights>) {
      auto& random_engine = ranges::detail::get_random_engine();
      return low_variance_sample_from_range(std::forward<Range>(range), std::forward<Weights>(weights), random_engine);
    } else {
      static_assert(ranges::range<Range>);
      static_assert(std::is_lvalue_reference_v<Weights&&>);
      return low_variance_sample_from_range(std::forward<Range>(range), weights);
    }
  }

  /// Overload that takes one argument.
  template <class Range>
  constexpr auto operator()(Range&& range) const {
    if constexpr (ranges::range<Range>) {
      auto& random_engine = ranges::detail::get_random_engine();
      return low_variance_sample_from_range(std::forward<Range>(range), random_engine);
    } else {
      static_assert(std::is_lvalue_reference_v<Range&&>);
      return ranges::make_view_closure(ranges::bind_back(low_variance_sample_fn{}, std::ref(range)));
    }
  }

  /// Overload that unwraps the engine reference from a view closure.
  template <class Range, class RandomEngine>
  constexpr auto operator()(Range&& range, std::reference_wrapper<RandomEngine> random_engine) const {
    static_assert(ranges::range<Range>);
    return low_variance_sample_from_range(std::forward<Range>(range), random_engine.get());
  }
};

}  // namespace detail
/// \brief A view adaptor that samples elements from a range with replacement using low variance sampling.

inline constexpr ranges::views::view_closure<detail::low_variance_sample_fn> low_variance_sample;

}  // namespace beluga::views

#endif
