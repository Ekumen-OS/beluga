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

#ifndef BELUGA_AMCL__LASER_SCAN_HPP_
#define BELUGA_AMCL__LASER_SCAN_HPP_

#include <algorithm>
#include <cassert>
#include <limits>
#include <memory>
#include <utility>

#include <beluga/sensor/scan.hpp>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/stride.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <sophus/se2.hpp>

namespace beluga_amcl {

/// Generates a vector of points to update the measurement model.
/**
 * \param laser_scan Laser scan message to process.
 * \param laser_transform Transform from laser frame to robot frame.
 * \param max_beam_count Maximum number of evenly-spaced beams to be taken from the scan.
 * \param range_min Minimum range value. Beams with a range below this value will be ignored.
 * \param range_max Maximum range value. Beams with a range above this value will be ignored.
 * \return A vector of pairs of coordinates in the robot's reference frame.
 */
class LaserScan : public beluga::RotatingBeamScan<LaserScan> {
 public:
  using Scalar = double;

  explicit LaserScan(
      std::shared_ptr<sensor_msgs::msg::LaserScan> scan,
      std::size_t max_beams = std::numeric_limits<std::size_t>::max(),
      Sophus::SE2d origin = Sophus::SE2d{})
    : scan_(std::move(scan)), max_beams_(max_beams), origin_(std::move(origin))
  {
    assert(scan != nullptr);
  }

  const Sophus::SE2d& origin() const { return origin_; }

  std::size_t size() const { return std::min(scan_->ranges.size(), max_beams_); }

  auto angles() const {
    return
        ranges::views::ints(0, static_cast<int>(scan_->ranges.size())) |
        ranges::views::stride(stride()) | ranges::views::take(size()) |
        ranges::views::transform([this](int i) {
          return static_cast<double>(
              scan_->angle_min + static_cast<float>(i) * scan_->angle_increment);
        });
  }

  auto ranges() const {
    return scan_->ranges | ranges::views::stride(stride()) | ranges::views::take(size()) |
        ranges::views::transform([] (auto value) { return static_cast<double>(value); });
  }

  auto range_bounds() const {
    return std::make_pair(
        static_cast<double>(scan_->range_min),
        static_cast<double>(scan_->range_max));
  }

 private:
  std::size_t stride() const {
    if (scan_->ranges.size() > 1UL && max_beams_ > 1UL) {
      return std::max(1UL, (scan_->ranges.size() - 1UL) / (max_beams_ - 1UL));
    }
    return 1UL;
  }

  std::shared_ptr<sensor_msgs::msg::LaserScan> scan_;
  std::size_t max_beams_;
  Sophus::SE2d origin_;
};

}  // namespace beluga_amcl

#endif
