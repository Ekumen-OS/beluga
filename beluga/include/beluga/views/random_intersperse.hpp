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

#ifndef BELUGA_VIEWS_RANDOM_INTERSPERSE_HPP
#define BELUGA_VIEWS_RANDOM_INTERSPERSE_HPP

#include <functional>
#include <optional>
#include <random>
#include <type_traits>

#include <range/v3/functional/bind_back.hpp>
#include <range/v3/utility/random.hpp>
#include <range/v3/view/adaptor.hpp>

/**
 * \file
 * \brief Implementation of a random_intersperse range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// Implementation of the random_intersperse view as a view adaptor.
/**
 * \tparam Range A [forward range](https://en.cppreference.com/w/cpp/ranges/forward_range).
 * \tparam Fn A callable type which takes no arguments and returns values to be inserted.
 * \tparam URNG A random number generator that satisfies the
 *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
 *  requirements.
 */
template <class Range, class Fn, class URNG = typename ranges::detail::default_random_engine>
struct random_intersperse_view
    : public ranges::view_adaptor<
          random_intersperse_view<Range, Fn, URNG>,
          Range,
          // The cardinality value is unknown at compile time.
          // If the adapted range cardinality is finite then we know the resulting view is finite.
          // But the intersperse probability could be 1.0, leading to an infinite range in practice.
          ranges::unknown> {
 public:
  /// Default constructor.
  random_intersperse_view() = default;

  /// Construct the view from a range to be adapted.
  /**
   * \param range The range to be adapted.
   * \param fn The generator function that returns values to be inserted.
   * \param probability The probability of inserting a value on each iteration.
   * \param engine The random number generator object.
   */
  constexpr random_intersperse_view(
      Range range,
      Fn fn,
      double probability,
      URNG& engine = ranges::detail::get_random_engine())
      : random_intersperse_view::view_adaptor{std::move(range)},
        fn_{std::move(fn)},
        distribution_{probability},
        engine_{std::addressof(engine)} {}

 private:
  // `ranges::range_access` needs access to the adaptor members.
  friend ranges::range_access;

  using result_type = ranges::common_type_t<decltype(std::declval<Fn>()()), ranges::range_value_t<Range>>;

  /// Adaptor subclass that implements the random_intersperse logic.
  struct adaptor : public ranges::adaptor_base {
   public:
    /// Default constructor.
    adaptor() = default;

    /// Construct an iterator adaptor from the parent view.
    constexpr explicit adaptor(random_intersperse_view* view) noexcept : view_(view) {}

    /// Return the inserted value or dereference the current iterator.
    [[nodiscard]] constexpr auto read(ranges::iterator_t<Range> it) const { return fn_return_.value_or(*it); }

    /// Generate a new value to be inserted or increment the input iterator.
    constexpr void next(ranges::iterator_t<Range>& it) {
      fn_return_.reset();
      if (view_->should_intersperse()) {
        fn_return_ = view_->fn_();
      } else {
        ++it;
      }
    }

    void prev(ranges::iterator_t<Range>& it) = delete;
    void advance() = delete;
    void distance_to() = delete;

   private:
    random_intersperse_view* view_;
    std::optional<result_type> fn_return_;
  };

  /// Return the adaptor for the begin iterator.
  [[nodiscard]] constexpr auto begin_adaptor() { return adaptor{this}; }

  /// Return whether we should intersperse a value or increment the input iterator.
  [[nodiscard]] constexpr bool should_intersperse() { return distribution_(*engine_); }

  ranges::semiregular_box_t<Fn> fn_;
  std::bernoulli_distribution distribution_;
  URNG* engine_;
};

/// Implementation detail for a random_intersperse range adaptor object.
struct random_intersperse_fn {
  /// Default insertion probability on each iteration.
  static constexpr double kDefaultProbability = 0.5;

  /// Overload that implements the andom_intersperse algorithm.
  /**
   * \tparam Range A [forward range](https://en.cppreference.com/w/cpp/ranges/forward_range).
   * \tparam Fn A callable type which takes no arguments or a distribution type that takes a URNG.
   * \tparam URNG A random number generator that satisfies the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param range The range to be adapted.
   * \param fn Fn instance used to insert values between source elements.
   * \param probability The probability of inserting a value on each iteration.
   * \param engine The random number generator object.
   */
  template <class Range, class Fn, class URNG = typename ranges::detail::default_random_engine>
  constexpr auto operator()(
      Range&& range,
      Fn fn,
      double probability = kDefaultProbability,
      URNG& engine = ranges::detail::get_random_engine()) const {
    // Support nullary function objects and distribution-like objects (that take a URNG).
    auto gen = [&fn, &engine]() {
      if constexpr (std::is_invocable_v<Fn>) {
        (void)(engine);  // Not used.
        return std::move(fn);
      } else {
        static_assert(std::is_invocable_v<Fn, decltype(engine)>);
        return [fn = std::move(fn), &engine]() { return fn(engine); };
      }
    }();

    return random_intersperse_view{ranges::views::all(std::forward<Range>(range)), std::move(gen), probability, engine};
  }

  /// Overload that unwraps the engine reference from a view closure.
  template <class Range, class Fn, class URNG>
  constexpr auto operator()(Range&& range, Fn fn, double probability, std::reference_wrapper<URNG> engine) const {
    return (*this)(ranges::views::all(std::forward<Range>(range)), std::move(fn), probability, engine.get());
  }

  /// Overload that returns a view closure to compose with other views.
  /**
   * \tparam Fn A callable type which takes no arguments and returns values to be inserted.
   * \tparam URNG A random number generator that satisfies the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param fn Fn instance used to insert values between source elements.
   * \param probability The probability of inserting a value on each iteration.
   * \param engine The random number generator object.
   */
  template <class Fn, class URNG = typename ranges::detail::default_random_engine>
  constexpr auto operator()(
      Fn fn,
      double probability = kDefaultProbability,
      URNG& engine = ranges::detail::get_random_engine()) const {
    return ranges::make_view_closure(
        ranges::bind_back(random_intersperse_fn{}, std::move(fn), probability, std::ref(engine)));
  }
};

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will insert values from a generator function between contiguous elements from the source based
/// on a given probability.
inline constexpr detail::random_intersperse_fn random_intersperse;

}  // namespace beluga::views

#endif
