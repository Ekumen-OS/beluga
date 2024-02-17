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

#ifndef BELUGA_VIEWS_SAMPLE_HPP
#define BELUGA_VIEWS_SAMPLE_HPP

#include <random>

#include <range/v3/utility/random.hpp>
#include <range/v3/view/common.hpp>
#include <range/v3/view/generate.hpp>

#include <beluga/type_traits/particle_traits.hpp>
#include <beluga/views/particles.hpp>

#include <beluga/detail/prologue.hpp>

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
 * \tparam Distribution A random number distribution that satisfies the
 *  [RandomNumberDistribution](https://en.cppreference.com/w/cpp/named_req/RandomNumberDistribution).
 * \tparam URNG A random number generator that satisfies the
 *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
 *  requirements.
 */
template <class Range, class Distribution, class URNG = typename ranges::detail::default_random_engine>
struct sample_view : public ranges::view_facade<sample_view<Range, Distribution, URNG>, ranges::infinite> {
 public:
  /// Default constructor.
  sample_view() = default;

  /// Construct the view from an existing range.
  /**
   * \param range The range to be adapted.
   * \param distribution The random number distribution to use to sample elements.
   * \param engine The random number generator object.
   */
  constexpr sample_view(Range range, Distribution distribution, URNG& engine = ranges::detail::get_random_engine())
      : range_{std::move(range)},
        distribution_{std::make_shared<Distribution>(std::move(distribution))},
        engine_{std::addressof(engine)} {
    assert(ranges::size(range) > 0);
    assert(distribution_->min() == 0);
    assert(distribution_->max() == static_cast<typename Distribution::result_type>(ranges::size(range_)) - 1);
  }

 private:
  // `ranges::range_access` needs access to the cursor members.
  friend ranges::range_access;

  static_assert(ranges::sized_range<Range>);
  static_assert(ranges::random_access_range<Range>);
  static_assert(std::is_same_v<typename Distribution::result_type, ranges::range_difference_t<Range>>);

  /// Cursor class that handles the iteration logic.
  struct cursor {
   public:
    /// Default constructor.
    cursor() = default;

    /// Construct a cursor from the parent view elements.
    constexpr cursor(std::shared_ptr<Distribution> distribution, URNG* engine, ranges::iterator_t<Range> first)
        : distribution_{std::move(distribution)}, engine_{engine}, first_{std::move(first)} {
      assert(distribution_ != nullptr);
      next();
    }

    /// Access the current iterator.
    [[nodiscard]] decltype(auto) read() const noexcept(noexcept(*this->it_)) { return *it_; }

    /// Position the current iterator.
    void next() { it_ = first_ + (*distribution_)(*engine_); }

   private:
    std::shared_ptr<Distribution> distribution_;
    URNG* engine_;
    ranges::iterator_t<Range> first_;
    ranges::iterator_t<Range> it_;
  };

  /// Return the cursor for the begin iterator.
  [[nodiscard]] constexpr auto begin_cursor() { return cursor{distribution_, engine_, ranges::begin(range_)}; }

  /// Return an unreachable sentinel since this is an infinite range.
  [[nodiscard]] constexpr auto end_cursor() const noexcept { return ranges::unreachable_sentinel_t{}; }

  Range range_;

  // Using a shared resource as the current version of range-v3 does not support move-only views.
  // This is to ensure the O(1) (cheaply) copyable semantic requirement for views.
  std::shared_ptr<Distribution> distribution_;
  URNG* engine_;
};

/// \cond

template <class T, class Enable = void>
struct is_random_distribution : public std::false_type {};

template <class T>
struct is_random_distribution<T, std::void_t<decltype(std::declval<T&>()(std::declval<std::mt19937&>()))>>
    : std::true_type {};

template <class T>
inline constexpr bool is_random_distribution_v = is_random_distribution<T>::value;

/// \endcond

/// Implementation detail for a sample algorithm.
struct sample_base_fn {
 protected:
  /// Sample from weighted ranges.
  template <class Range, class Weights, class URNG>
  constexpr auto sample_from_range(Range&& range, Weights&& weights, URNG& engine) const {
    static_assert(ranges::sized_range<Range>);
    static_assert(ranges::random_access_range<Range>);
    static_assert(ranges::input_range<Weights>);
    using result_type = ranges::range_difference_t<Range>;
    auto w = ranges::views::common(weights);
    auto distribution = std::discrete_distribution<result_type>{ranges::begin(w), ranges::end(w)};
    return sample_view{ranges::views::all(std::forward<Range>(range)), std::move(distribution), engine};
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
  constexpr auto sample_from_range(Range&& range, URNG& engine) const {
    static_assert(ranges::sized_range<Range>);
    static_assert(ranges::random_access_range<Range>);
    if constexpr (beluga::is_particle_range_v<Range>) {
      return sample_from_range(beluga::views::states(range), beluga::views::weights(range), engine) |
             ranges::views::transform(beluga::make_from_state<ranges::range_value_t<Range>>);
    } else {
      using result_type = ranges::range_difference_t<Range>;
      auto distribution =
          std::uniform_int_distribution<result_type>{0, static_cast<result_type>(ranges::size(range) - 1)};
      return sample_view{ranges::views::all(std::forward<Range>(range)), std::move(distribution), engine};
    }
  }

  /// Sample from random distributions.
  template <class Distribution, class URNG>
  constexpr auto sample_from_distribution(Distribution distribution, URNG& engine) const {
    return ranges::views::generate(
        [distribution = std::move(distribution), &engine]() mutable { return distribution(engine); });
  }
};

/// Implementation detail for a sample range adaptor object.
struct sample_fn : public sample_base_fn {
  /// Overload that takes three arguments.
  template <class T, class U, class V>
  constexpr auto operator()(T&& t, U&& u, V& v) const {
    static_assert(ranges::range<T>);
    static_assert(ranges::range<U>);
    return sample_from_range(std::forward<T>(t), std::forward<U>(u), v);  // Assume V is a URNG
  }

  /// Overload that takes two arguments.
  template <class T, class U>
  constexpr auto operator()(T&& t, U&& u) const {
    if constexpr (ranges::range<T> && ranges::range<U>) {
      auto& engine = ranges::detail::get_random_engine();
      return sample_from_range(std::forward<T>(t), std::forward<U>(u), engine);
    } else if constexpr (is_random_distribution_v<T>) {
      static_assert(std::is_lvalue_reference_v<U&&>);  // Assume U is a URNG
      return sample_from_distribution(std::forward<T>(t), u);
    } else {
      static_assert(ranges::range<T>);
      static_assert(std::is_lvalue_reference_v<U&&>);  // Assume U is a URNG
      return sample_from_range(std::forward<T>(t), u);
    }
  }

  /// Overload that takes one argument.
  template <class T>
  constexpr auto operator()(T&& t) const {
    if constexpr (ranges::range<T>) {
      auto& engine = ranges::detail::get_random_engine();
      return sample_from_range(std::forward<T>(t), engine);
    } else if constexpr (is_random_distribution_v<T>) {
      auto& engine = ranges::detail::get_random_engine();
      return sample_from_distribution(std::forward<T>(t), engine);
    } else {
      static_assert(std::is_lvalue_reference_v<T&&>);  // Assume T is a URNG
      return ranges::make_view_closure(ranges::bind_back(sample_fn{}, std::ref(t)));
    }
  }

  /// Overload that unwraps the engine reference from a view closure.
  template <class Range, class URNG>
  constexpr auto operator()(Range&& range, std::reference_wrapper<URNG> engine) const {
    static_assert(ranges::range<Range>);
    return sample_from_range(std::forward<Range>(range), engine.get());
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
 */
inline constexpr ranges::views::view_closure<detail::sample_fn> sample;

}  // namespace beluga::views

namespace ranges {

/// \cond

/// Enable borrowed range specialization.
/**
 * A function can take this range by value and return iterators obtained from it without danger of dangling,
 * as long as the input range is also a borrowed range.
 */
template <class Range, class Distribution, class URNG>
inline constexpr bool enable_borrowed_range<beluga::views::detail::sample_view<Range, Distribution, URNG>> =
    enable_borrowed_range<Range>;

/// \endcond

}  // namespace ranges

#include <beluga/detail/epilogue.hpp>

#endif
