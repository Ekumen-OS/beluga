// Copyright 2024 Ekumen, Inc.
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

#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>

/**
 * \file
 * \brief Implementation of a sample (with replacement) range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// Implementation of the sample view.
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
          range_begin_{ranges::begin(view_->range_)},
          weights_begin_{ranges::begin(view_->weights_)},
          M_{static_cast<long long>(ranges::size(view_->range_))},
          r_{std::uniform_real_distribution<double>(0.0, 1.0 / static_cast<double>(M_))(
              *view_->engine_)},  // Generate r only once
          m_{0},
          i_{0},
          c_{*weights_begin_} {}

    /// Access the current iterator.
    [[nodiscard]] constexpr decltype(auto) read() const noexcept(noexcept(*range_begin_)) { return *it_; }

    /// Position the current iterator.
    constexpr void next() {
      ++m_;
      const double U = r_ + (static_cast<double>(m_) - 1.0) / static_cast<double>(M_);
      while (U > c_) {
        ++i_;
        c_ += static_cast<double>(*std::next(weights_begin_, i_));  // weights_begin_[i_]
      }
      it_ = std::next(range_begin_, i_);  // range_begin_ + i_
    }

    /// Returns true if the cursor is at the end of the range.
    constexpr bool equal(const cursor& other) const noexcept { return m_ == other.M_; }

   private:
    low_variance_sample_view* view_;
    ranges::iterator_t<Range> range_begin_;
    ranges::iterator_t<Range> it_;
    ranges::iterator_t<Weights> weights_begin_;
    long long M_;
    double r_;
    long long m_;
    long long i_;
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
      auto uniform_weights =
          ranges::views::repeat_n(1.0, ranges::size(range));  // Use 1.0 for double weights. Important.
      return low_variance_sample_from_range(std::forward<Range>(range), uniform_weights, engine);
    }
  }

  // /// Sample from random distributions.
  // template <class Distribution, class URNG>
  // constexpr auto low_variance_sample_from_distribution(Distribution distribution, URNG& engine) const {
  //   return ranges::views::generate(
  //       [distribution = std::move(distribution), &engine]() mutable { return distribution(engine); });
  // }
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
      // } else if constexpr (is_random_distribution_v<T>) {
      //   static_assert(std::is_lvalue_reference_v<U&&>);  // Assume U is a URNG
      //   return low_variance_sample_from_distribution(std::forward<T>(t), u);
    } else {
      static_assert(ranges::range<T>);
      static_assert(std::is_lvalue_reference_v<U&&>);  // Assume U is a URNG
      return low_variance_sample_from_range(std::forward<T>(t), u);
    }
  }

  /// Overload that takes one argument.
  template <class T>
  constexpr auto operator()(T&& t) const {
    if constexpr (ranges::range<T>) {
      auto& engine = ranges::detail::get_random_engine();
      return low_variance_sample_from_range(std::forward<T>(t), engine);
      // } else if constexpr (is_random_distribution_v<T>) {
      //   auto& engine = ranges::detail::get_random_engine();
      //   return low_variance_sample_from_distribution(std::forward<T>(t), engine);
    } else {
      static_assert(std::is_lvalue_reference_v<T&&>);  // Assume T is a URNG
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

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will randomly sample with replacement from an input range.
/**
 * Unlike `std::views::sample`, this does not require a size parameter and the samples will be taken
 * from the population **with replacement**, making the sample values independent.
 * To use this, the input range must model the
 * [random_access_range](https://en.cppreference.com/w/cpp/ranges/random_access_range)
 * and [sized_range](https://en.cppreference.com/w/cpp/ranges/sized_range) concepts.
 *
 * This view implements multinomial resampling for a given range of particles.
 * The core idea is to draw random indices / iterators to the input particle range
 * from a [multinomial distribution](https://en.wikipedia.org/wiki/Multinomial_distribution)
 * parameterized after particle weights (and assumed uniform for non-weighted particle ranges).
 *
 * This view can also be used to convert any random distribution (a callable that takes a URNG as an
 * input argument) into an infinite view that generates values from that distribution.
 *
 * This view is not cheap to copy, so care must be taken when moving it around.
 * Range-v3 does not support move-only views at the time of this implementation.
 */
inline constexpr ranges::views::view_closure<detail::low_variance_sample_fn> low_variance_sample;

}  // namespace beluga::views

#endif
