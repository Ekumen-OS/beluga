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

#ifndef BELUGA_ALGORITHM_RAYCASTING_BRESENHAM_HPP
#define BELUGA_ALGORITHM_RAYCASTING_BRESENHAM_HPP

#include <utility>

#include <range/v3/view/all.hpp>
#include <range/v3/view/take_while.hpp>

#include <Eigen/Core>

/**
 * \file
 * \brief Implementation of Bresenham-like raytracing algorithms.
 */

namespace beluga {

/// Bresenham's 2D line drawing algorithm, optimized for integer arithmetic.
class Bresenham2i {
 public:
  /// Bresenham's 2D line drawing algorithm variants.
  enum Variant {
    kStandard = 0,  ///< Standard Bresenham's algorithm.
    kModified       ///< Modified, aka supercover, Bresenham's algorithm.
                    ///  See http://eugen.dedu.free.fr/projects/bresenham.
  };

  /// Bresenham's 2D line drawing as a range.
  /**
   * \tparam Vector2 2D vector type. Must be default constructible,
   *   copy constructible, and implement `Vector2d::x()` and `Vector2::y()`
   *   methods returning both lvalues (for mutation) and rvalues.
   * \tparam Integer Integer scalar type.
   */
  template <class Vector2, typename Integer = typename Vector2::Scalar>
  class Line : public ranges::view_interface<Line<Vector2, Integer>> {
   public:
    /// Bresenham's 2D line drawing iterator, one cell at a time.
    class iterator {  // NOLINT(readability-identifier-naming)
     public:
      /// Past-of-end iterator sentinel.
      struct sentinel {
        /// Equality operator overload, for symmetry (as required by ranges::sentinel_for).
        bool operator==(const iterator& other) const { return other == *this; }

        /// Inequality operator overload, for symmetry (as required by ranges::sentinel_for).
        bool operator!=(const iterator& other) const { return !(other == *this); }
      };

      /// Iterator category tag.
      using iterator_category = std::forward_iterator_tag;

      /// Iterator difference type (as required by ranges::view_).
      using difference_type = std::ptrdiff_t;

      /// Iterated value type.
      using value_type = Vector2;

      /// Pointer to iterated value type.
      using pointer = Vector2*;

      /// Reference to iterated value type.
      using reference = Vector2&;

      /// Default constructor.
      iterator() = default;

      /// Default copy constructor.
      iterator(const iterator& other) = default;

      /// Default move constructor.
      iterator(iterator&&) = default;  // NOLINT(performance-noexcept-move-constructor)

      /// Default copy assignment operator overload.
      iterator& operator=(const iterator&) = default;

      /// Default move assignment operator overload.
      iterator& operator=(iterator&&) = default;  // NOLINT(performance-noexcept-move-constructor)

      /// Constructs a Bresenham's 2D `line` iterator.
      explicit iterator(const Line* line) : current_point_(line->p0_), x_(line->p0_.x()), y_(line->p0_.y()) {
        xspan_ = line->p1_.x() - line->p0_.x();
        xstep_ = static_cast<decltype(xspan_)>(1);
        if (xspan_ < 0) {
          xspan_ = -xspan_;
          xstep_ = -xstep_;
        }

        yspan_ = line->p1_.y() - line->p0_.y();
        ystep_ = static_cast<decltype(yspan_)>(1);
        if (yspan_ < 0) {
          yspan_ = -yspan_;
          ystep_ = -ystep_;
        }

        if (xspan_ < yspan_) {
          using std::swap;
          swap(x_, y_);
          swap(xspan_, yspan_);
          swap(xstep_, ystep_);
          reversed_ = true;
        }

        dxspan_ = 2 * xspan_;
        dyspan_ = 2 * yspan_;

        error_ = prev_error_ = xspan_;
        modified_ = line->variant_ == Bresenham2i::kModified;
      }

      /// Post-fix operator overload.
      iterator operator++(int) {
        iterator other = *this;
        this->operator++();
        return other;
      }

      /// Prefix operator overload.
      iterator& operator++() {
        if (checks_ == 0) {
          if (++step_ > xspan_) {
            return *this;
          }
          x_ += xstep_;
          error_ += dyspan_;
          ++checks_;
          if (error_ > dxspan_) {
            y_ += ystep_;
            error_ -= dxspan_;
            if (modified_) {
              ++checks_;
              ++checks_;
            }
          }
        }

        if (checks_ > 1) {
          if (checks_ > 2) {
            --checks_;
            if (error_ + prev_error_ <= dxspan_) {
              step_to(x_, y_ - ystep_);
              return *this;
            }
          }

          --checks_;
          if (error_ + prev_error_ >= dxspan_) {
            step_to(x_ - xstep_, y_);
            return *this;
          }
        }

        --checks_;
        step_to(x_, y_);
        prev_error_ = error_;
        return *this;
      }

      /// Dereference operator overload (only const).
      const Vector2& operator*() const { return current_point_; }

      /// Arrow operator overload (only const).
      const Vector2* operator->() const { return &current_point_; }

      /// Equality operator overload (as required by std::forward_iterator).
      bool operator==(const iterator& other) const {
        return x_ == other.x_ && y_ == other.y_ && xstep_ == other.xstep_ && ystep_ == other.ystep_ &&
               xspan_ == other.xspan_ && yspan_ == other.yspan_ && step_ == other.step_ && checks_ == other.checks_ &&
               modified_ == other.modified_ && reversed_ == other.reversed_;
      }

      /// Inequality operator overload (as required by std::forward_iterator).
      bool operator!=(const iterator& other) const { return !(*this == other); }

      /// Sentinel equality operator overload.
      bool operator==(const sentinel&) const { return step_ > xspan_; }

      /// Sentinel inequality operator overload.
      bool operator!=(const sentinel& other) const { return !(*this == other); }

     private:
      void step_to(Integer x, Integer y) {
        if (reversed_) {
          using std::swap;
          swap(x, y);
        }
        current_point_.x() = x;
        current_point_.y() = y;
      }

      Vector2 current_point_{};
      Integer x_{}, y_{};
      Integer xspan_{}, yspan_{};
      Integer dxspan_, dyspan_{};
      Integer xstep_{}, ystep_{}, step_{};
      Integer prev_error_{}, error_{};

      std::size_t checks_{0};
      bool modified_{false};
      bool reversed_{false};
    };

    /// Constructs point line.
    Line() = default;

    /// Default copy constructor.
    Line(const Line&) = default;

    /// Default move constructor.
    Line(Line&&) = default;  // NOLINT(performance-noexcept-move-constructor)

    /// Default copy assignment operator overload.
    Line& operator=(const Line&) = default;

    /// Default move assignment operator overload.
    Line& operator=(Line&&) = default;  // NOLINT(performance-noexcept-move-constructor)

    /// Constructs a Bresenham's 2D line drawing.
    /**
     * \param p0 Line start point in 2D space.
     * \param p1 Line end point in 2D space.
     * \param variant Bresenham's algorithm variant to be used.
     */
    explicit Line(Vector2 p0, Vector2 p1, Variant variant)
        : p0_(std::move(p0)), p1_(std::move(p1)), variant_(variant) {}

    /// Returns an iterator pointing to the first point in the line.
    [[nodiscard]] auto begin() const { return Line::iterator{this}; }

    /// Returns a sentinel as past-of-end iterator.
    [[nodiscard]] auto end() const { return typename Line::iterator::sentinel{}; }

   private:
    friend class iterator;

    Vector2 p0_{};
    Vector2 p1_{};
    Variant variant_{};
  };

  /// Constructs standard Bresenham 2D line drawing algorithm.
  Bresenham2i() noexcept = default;

  /// Default copy constructor.
  Bresenham2i(const Bresenham2i&) noexcept = default;

  /// Default move constructor.
  Bresenham2i(Bresenham2i&&) noexcept = default;

  /// Default copy assignment operator overload.
  Bresenham2i& operator=(const Bresenham2i&) noexcept = default;

  /// Default move assignment operator overload.
  Bresenham2i& operator=(Bresenham2i&&) noexcept = default;

  /// Constructs specific Bresenham 2D line drawing algorithm `variant`.
  explicit Bresenham2i(Variant variant) noexcept : variant_(variant) {}

  /// Computes 2D line from `p0` to `p1`.
  /**
   * \tparam Vector2i Point in 2D integer space ie. ℤ × ℤ.
   * \param p0 Start point in 2D integer space.
   * \param p1 End point in 2D integer space.
   * \return subtended bresenham2i::line.
   */
  template <class Vector2i = Eigen::Vector2i>
  auto operator()(Vector2i p0, Vector2i p1) const {
    return Line{std::move(p0), std::move(p1), variant_};
  }

 private:
  Variant variant_{};
};

}  // namespace beluga

#endif
