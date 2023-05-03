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

#ifndef BELUGA_ALGORITHM_RAYCASTING_HPP
#define BELUGA_ALGORITHM_RAYCASTING_HPP
#include <iostream>
#include <optional>

#include <range/v3/view/all.hpp>
#include <range/v3/view/take_while.hpp>

#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

/**
 * \file
 * \brief Implementation of algorithms to perform raycasting.
 */

namespace beluga {

/// Bresenham's 2D line drawing algorithm, optimized for integer arithmetic.
class bresenham2i {
 public:
  /// Bresenham's 2D line drawing algorithm variants.
  enum Variant {
    STANDARD = 0,  ///< Standard Bresenham's algorithm.
    MODIFIED       ///< Modified, aka supercover, Bresenham's algorithm.
  };

  /// Bresenham's 2D line drawing as a range.
  template <class Vector2, typename Integer = typename Vector2::Scalar>
  class line : public ranges::view_interface<line<Vector2, Integer>> {
   public:
    /// Bresenham's 2D line drawing iterator, one cell at a time.
    class iterator {
     public:
      struct sentinel {
        bool operator==(const iterator& other) const { return other == *this; }
        bool operator!=(const iterator& other) const { return !(other == *this); }
      };

      using iterator_category = std::forward_iterator_tag;
      using difference_type = std::ptrdiff_t;
      using value_type = Vector2;
      using pointer = Vector2*;
      using reference = Vector2&;

      iterator() = default;
      iterator(const iterator&) = default;
      iterator(iterator&&) = default;
      iterator& operator=(const iterator&) = default;
      iterator& operator=(iterator&&) = default;

      explicit iterator(const line* line) : p_(line->p0_), x_(line->p0_.x()), y_(line->p0_.y()) {
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
        modified_ = line->variant_ == bresenham2i::MODIFIED;
      }

      iterator operator++(int) {
        iterator other = *this;
        this->operator++();
        return other;
      }

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

      const Vector2& operator*() const { return p_; }

      const Vector2* operator->() const { return &p_; }

      bool operator==(const iterator& other) const {
        return x_ == other.x_ && y_ == other.y_ && xstep_ == other.xstep_ && ystep_ == other.ystep_ &&
               xspan_ == other.xspan_ && yspan_ == other.yspan_ && step_ == other.step_ && checks_ == other.checks_ &&
               modified_ == other.modified_ && reversed_ == other.reversed_;
      }

      bool operator!=(const iterator& other) const { return !(*this == other); }

      bool operator==(const sentinel&) const { return step_ > xspan_; }

      bool operator!=(const sentinel& other) const { return !(*this == other); }

     private:
      void step_to(Integer x, Integer y) {
        if (reversed_) {
          using std::swap;
          swap(x, y);
        }
        p_.x() = x;
        p_.y() = y;
      }

      Vector2 p_{};
      Integer x_{}, y_{};
      Integer xspan_{}, yspan_{};
      Integer dxspan_, dyspan_{};
      Integer xstep_{}, ystep_{}, step_{};
      Integer prev_error_{}, error_{};

      std::size_t checks_{0};
      bool modified_{false};
      bool reversed_{false};
    };

    line() = default;
    line(const line&) = default;
    line(line&&) = default;
    line& operator=(const line&) = default;
    line& operator=(line&&) = default;

    /// Constructs a Bresenham's 2D line drawing.
    /**
     * \param _p0 Line start point in 2D space.
     * \param _p1 Line end point in 2D space.
     * \param _variant Bresenham's algorithm variant to be used.
     */
    explicit line(const Vector2& p0, const Vector2& p1, Variant variant)
      : p0_(p0), p1_(p1), variant_(variant) {}

    auto begin() const { return line::iterator{this}; }
    auto end() const { return typename line::iterator::sentinel{}; }

   private:
    friend class iterator;
    /// Line drawing start point.
    Vector2 p0_{};
    /// Line drawing end point.
    Vector2 p1_{};
    /// Bresenham's algorithm variant used.
    Variant variant_{};
  };

  bresenham2i() = default;
  bresenham2i(const bresenham2i&) = default;
  bresenham2i(bresenham2i&&) = default;
  bresenham2i& operator=(const bresenham2i&) = default;
  bresenham2i& operator=(bresenham2i&&) = default;

  /// Constructs specific Bresenham 2D line drawing `variant`.
  explicit bresenham2i(Variant variant) : variant_(variant) {}

  /// Computes 2D line from `p0` to `p1`.
  /**
   * \tparam Vector2i Point in 2D integer space ie. ℤ × ℤ.
   * \param p0 Start point in 2D integer space.
   * \param p1 End point in 2D integer space.
   * \return bresenham2i::line subtended.
   */
  template <class Vector2i = Eigen::Vector2i>
  auto operator()(const Vector2i& p0, const Vector2i& p1) const {
    return line{p0, p1, variant_};
  }

  /// Chosen algorithm variant.
 private:
  Variant variant_{};
};

/// 2D ray casting
/**
 * \tparam Grid A 2D grid
 * \tparam Algorithm A callable type, taking start and end
 *   grid cells for a ray and returning the full trace.
 */
template <class Grid, typename Algorithm = bresenham2i>
class ray2d {
 public:
  /// Constructs 2D ray with default algorithm.
  /**
   * See ray2d(const Grid &, Algorithm, const Sophus::SE2d&, double)
   * for further reference on constructor arguments.
   */
  ray2d(const Grid& grid, const Sophus::SE2d& source_pose, double max_range)
    : ray2d(grid, Algorithm{}, source_pose, max_range)
  {
  }

  /// Constructs 2D ray.
  /**
   * \param grid Grid on which to perform ray casting.
   * \param algorithm Ray tracing algorithm implementation.
   * \param source_pose Pose of the source of the ray
   *   in the same frame as that of the `grid` origin.
   * \param max_range Maximum range for the ray, in meters.
   */
  ray2d(const Grid& grid, Algorithm algorithm, const Sophus::SE2d& source_pose, double max_range)
    : grid_(grid), algorithm_(std::move(algorithm)),
      source_pose_in_grid_frame_(grid_.origin().inverse() * source_pose),
      max_range_(max_range)
  {
  }

  /// Computes ray trace along a given direction.
  /**
   * \param bearing Direction for ray tracing.
   * \return Full range of grid cells traced by the ray.
   *   That is, regardless of grid cells' state.
   */
  [[nodiscard]] auto trace(const Sophus::SO2d& bearing) const {
    const auto far_end_pose_in_source_frame = Sophus::SE2d{
        Sophus::SO2d{0.}, Eigen::Vector2d{
          max_range_ * std::cos(bearing.log()),
          max_range_ * std::sin(bearing.log())}};
    const auto far_end_pose_in_grid_frame =
        source_pose_in_grid_frame_ * far_end_pose_in_source_frame;
    const auto start_cell = grid_.cell(source_pose_in_grid_frame_.translation());
    const auto end_cell = grid_.cell(far_end_pose_in_grid_frame.translation());
    const auto cell_is_valid = [this](const auto& cell) { return grid_.valid(cell); };
    return algorithm_(start_cell, end_cell) | ranges::views::take_while(cell_is_valid);
  }

  /// Casts ray along a given direction.
  /**
   * Distances are measured from cell centroid to cell centroid.
   *
   * \param bearing Direction for ray casting.
   * \return Distance in meters to first occupied cell hit by the ray, if any.
   */
  [[nodiscard]] std::optional<double> cast(const Sophus::SO2d& bearing) const {
    const auto is_free = [this](const auto& cell) {
      // TODO(hidmic): move to occupancy grid API
      return Grid::Traits::is_free(grid_.data()[grid_.index(cell)]);
    };
    for (const auto& cell : trace(bearing)) {
      if (!is_free(cell)) {
        const auto start_cell = grid_.cell(source_pose_in_grid_frame_.translation());
        const auto distance = (grid_.point(cell) - grid_.point(start_cell)).norm();
        return std::make_optional(std::min(distance, max_range_));
      }
    }
    return std::nullopt;
  }

 private:
  const Grid& grid_;
  const Algorithm algorithm_;
  const Sophus::SE2d source_pose_in_grid_frame_;
  const double max_range_;
};

/// Leverages a Brasenham raycasting technique to cast a ray on the occupancy grid, \n
/// from the laser frame and with the provided bearing until \n
///  it hits an unknown/occupied cell or reaches `max_beam_range`.
/// Unknown cells are treated as occupied.
/**
 * \tparam OccupancyGrid Type that satisfies \ref OccupancyGridPage.
 * \param grid Grid to cast the ray on.
 * \param starting_position Position of the laser in map frame.
 * \param bearing_in_laser_frame An SO2 rotation that represents the bearing of the beam in laser frame.
 * \param max_beam_range Maximum range of the sensor in meters.
 * \return double Length in meters of the casted ray.
 */

}  // namespace beluga
#endif
