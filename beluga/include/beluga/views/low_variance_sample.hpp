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
/// This algorithm computes a single random number r in the range [0, 1/M) and then selects samples from
/// according to this number but still with a probability proportional to the sample weight. This is
/// done by drawing a random number r in the interval [0;M^-1]. Where M is the number of samples in the range.
/// It selects the particles by repeadly adding the fixed amount M^(-1) to r and by choosing the particle that
/// corresponds to to the resulting number.
/// Where M is the number of samples in the range.
/// [Based on] Sebastian Thrun, Wolfram Burgard, and Dieter Fox. 2005. Probabilistic Robotics (Intelligent Robotics
/// and Autonomous Agents). The MIT Press.

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
          i_{0},                  // Current index in the range.
          c_{*weights_begin_}     // Cumulative weight of the first particle
    {}

    /// Access the current iterator.
    [[nodiscard]] constexpr decltype(auto) read() const noexcept(noexcept(*this->it_)) { return *it_; }

    /// Position the current iterator.
    constexpr void next() {
      ++m_;
      // A number U in [0, 1] that points to exactly one particle in the range.
      // Where the particle i satisfies with i=argmin_j \sum_1^j w^m >= U.
      const double U = r_ + (static_cast<double>(m_) - 1.0) / static_cast<double>(M_);
      while (i_ < M_ - 1 && U > c_) {
        ++i_;
        c_ += static_cast<double>(*std::next(weights_begin_, i_));  // weights_begin_[i_]
      }
      it_ = std::next(range_begin_, i_);  // range_begin_ + i_
    }

   private:
    low_variance_sample_view* view_;
    ranges::iterator_t<Range> range_begin_;
    ranges::iterator_t<Range> it_;
    ranges::iterator_t<Weights> weights_begin_;
    uint64_t M_;
    double r_;
    uint64_t m_;
    uint64_t i_;
    double c_;
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
  template <class T, class U, class V>
  constexpr auto operator()(T&& t, U&& u, V& v) const {
    static_assert(ranges::range<T>);
    static_assert(ranges::range<U>);
    return low_variance_sample_from_range(std::forward<T>(t), std::forward<U>(u), v);  // Assume V is a URNG
  }

  /// Overload that takes two arguments.
  template <class T, class U>
  constexpr auto operator()(T&& t, U&& u) const {
    if constexpr (ranges::range<T> && ranges::range<U>) {
      auto& engine = ranges::detail::get_random_engine();
      return low_variance_sample_from_range(std::forward<T>(t), std::forward<U>(u), engine);
    } else {
      static_assert(ranges::range<T>);
      static_assert(std::is_lvalue_reference_v<U&&>);
      return low_variance_sample_from_range(std::forward<T>(t), u);
    }
  }

  /// Overload that takes one argument.
  template <class T>
  constexpr auto operator()(T&& t) const {
    if constexpr (ranges::range<T>) {
      auto& engine = ranges::detail::get_random_engine();
      return low_variance_sample_from_range(std::forward<T>(t), engine);
    } else {
      static_assert(std::is_lvalue_reference_v<T&&>);
      return ranges::make_view_closure(ranges::bind_back(low_variance_sample_fn{}, std::ref(t)));
    }
  }

  /// Overload that unwraps the engine reference from a view closure.
  template <class Range, class URNG>
  constexpr auto operator()(Range&& range, std::reference_wrapper<URNG> engine) const {
    static_assert(ranges::range<Range>);
    return low_variance_sample_from_range(std::forward<Range>(range), engine.get());
  }
};

}  // namespace detail
/// \brief A view adaptor that samples elements from a range with replacement using low variance sampling.

inline constexpr ranges::views::view_closure<detail::low_variance_sample_fn> low_variance_sample;

}  // namespace beluga::views

#endif
