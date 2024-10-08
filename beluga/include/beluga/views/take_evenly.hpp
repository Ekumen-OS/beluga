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

#ifndef BELUGA_VIEWS_TAKE_EVENLY_HPP
#define BELUGA_VIEWS_TAKE_EVENLY_HPP

#include <beluga/views/elements.hpp>

#include <range/v3/view/cache1.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>

/**
 * \file
 * \brief Implementation of a take_evenly range adaptor object.
 */

namespace beluga::views {

namespace detail {

/// \cond detail

template <class Range>
struct take_evenly_view : public ranges::view_facade<take_evenly_view<Range>, ranges::finite> {
 public:
  static_assert(ranges::view_<Range>);
  static_assert(ranges::forward_range<Range>);
  static_assert(ranges::sized_range<Range>);

  constexpr take_evenly_view() = default;

  constexpr take_evenly_view(Range range, std::size_t count)
      : range_{std::move(range)}, count_{count}, size_{ranges::size(range_)} {}

  [[nodiscard]] constexpr std::size_t size() const {
    if (size_ == 0U) {
      return 0U;
    }

    if (count_ > size_) {
      return size_;
    }

    return count_;
  }

 private:
  friend ranges::range_access;

  struct cursor {
   public:
    cursor() = default;

    constexpr cursor(const take_evenly_view* view, std::size_t pos)
        : view_{view}, pos_{pos}, it_{ranges::begin(view_->range_)} {}

    [[nodiscard]] constexpr decltype(auto) read() const { return *it_; }

    [[nodiscard]] constexpr bool equal(const cursor& other) const { return view_ == other.view_ && pos_ == other.pos_; }

    [[nodiscard]] constexpr bool equal(const ranges::default_sentinel_t&) const {
      return pos_ >= view_->count_ || pos_ >= view_->size_;
    }

    constexpr void next() {
      const std::size_t new_pos = pos_ + 1;
      ranges::advance(it_, view_->compute_offset(pos_, new_pos));
      pos_ = new_pos;
    }

    template <
        class T = Range,
        std::enable_if_t<std::is_same_v<T, Range> && ranges::bidirectional_range<Range>, int> = 0>
    constexpr void prev() {
      const std::size_t new_pos = pos_ - 1;
      ranges::advance(it_, view_->compute_offset(pos_, new_pos));
      pos_ = new_pos;
    }

    template <
        class T = Range,
        std::enable_if_t<std::is_same_v<T, Range> && ranges::random_access_range<Range>, int> = 0>
    constexpr void advance(std::ptrdiff_t distance) {
      const auto new_pos = static_cast<std::size_t>(static_cast<std::ptrdiff_t>(pos_) + distance);
      ranges::advance(it_, view_->compute_offset(pos_, new_pos));
      pos_ = new_pos;
    }

    template <
        class T = Range,
        std::enable_if_t<std::is_same_v<T, Range> && ranges::random_access_range<Range>, int> = 0>
    [[nodiscard]] constexpr std::ptrdiff_t distance_to(const cursor& other) const {
      return static_cast<std::ptrdiff_t>(other.pos_) - static_cast<std::ptrdiff_t>(pos_);
    }

    template <
        class T = Range,
        std::enable_if_t<std::is_same_v<T, Range> && ranges::random_access_range<Range>, int> = 0>
    [[nodiscard]] constexpr std::ptrdiff_t distance_to(const ranges::default_sentinel_t&) const {
      return static_cast<std::ptrdiff_t>(view_->count_) - static_cast<std::ptrdiff_t>(pos_);
    }

   private:
    const take_evenly_view* view_;
    std::size_t pos_;

    ranges::iterator_t<Range> it_;
  };

  [[nodiscard]] constexpr auto begin_cursor() const { return cursor{this, 0U}; }

  [[nodiscard]] constexpr auto end_cursor() const noexcept { return ranges::default_sentinel_t{}; }

  [[nodiscard]] constexpr std::ptrdiff_t compute_offset(std::size_t pos) const {
    assert(pos <= count_);

    if (pos == 0U) {
      return 0;
    }

    if (count_ == 1U) {
      return static_cast<std::ptrdiff_t>(size_);
    }

    const std::ptrdiff_t a = static_cast<std::ptrdiff_t>(pos) * (static_cast<std::ptrdiff_t>(size_) - 1);
    const std::ptrdiff_t b = static_cast<std::ptrdiff_t>(count_) - 1;
    return a / b + ((a % b == 0) ? 0 : 1);
  }

  [[nodiscard]] constexpr std::ptrdiff_t compute_offset(std::size_t current, std::size_t target) const {
    if (count_ > size_) {
      return static_cast<std::ptrdiff_t>(target) - static_cast<std::ptrdiff_t>(current);
    }

    return compute_offset(target) - compute_offset(current);
  }

  Range range_;
  std::size_t count_;
  std::size_t size_;
};

template <class Range>
take_evenly_view(Range&& range, std::size_t max) -> take_evenly_view<ranges::views::all_t<Range>>;

struct take_evenly_fn {
  template <class Range>
  constexpr auto operator()(Range&& range, std::size_t count) const {
    return take_evenly_view{std::forward<Range>(range), count};
  }

  constexpr auto operator()(std::size_t count) const {
    return ranges::make_view_closure(ranges::bind_back(take_evenly_fn{}, count));
  }
};

/// \endcond

}  // namespace detail

/// [Range adaptor object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject) that
/// will take elements evenly spaced over a range.
/**
 * Given a source range and an integral count, returns a range consisting of `count`
 * elements evenly spaced over the source range.
 * If `count` or the range size are zero, it returns an empty range.
 * If `count` is greater than the range size, it returns all the elements.
 * The first element of the range is always included.
 */
inline constexpr detail::take_evenly_fn take_evenly;

}  // namespace beluga::views

#endif
