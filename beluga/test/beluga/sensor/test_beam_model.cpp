#include "beluga/algorithm/raycasting.hpp"
#include "beluga/sensor/beam_model.hpp"
#include "beluga/sensor.hpp"
#include "ciabatta/ciabatta.hpp"
#include <gmock/gmock.h>

namespace beluga{

template <std::size_t Rows, std::size_t Cols>
class StaticOccupancyGrid {
 public:
  struct Traits {
    static bool is_free(bool value) { return !value; }
    static bool is_unknown(bool) { return false; }
    static bool is_occupied(bool value) { return value; }
  };

  explicit StaticOccupancyGrid(
      std::array<bool, Rows * Cols> array,
      double resolution = 1.0,
      const Sophus::SE2d& origin = Sophus::SE2d{})
      : grid_{array}, origin_{origin}, origin_inverse_{origin.inverse()}, resolution_{resolution} {}

  [[nodiscard]] std::size_t size() const { return grid_.size(); }

  [[nodiscard]] const auto& data() const { return grid_; }

  [[nodiscard]] const Sophus::SE2d& origin() const { return origin_; }

  [[nodiscard]] const Sophus::SE2d& origin_inverse() const { return origin_inverse_; }

  [[nodiscard]] std::size_t index(double x, double y) const {
    const auto x_index = static_cast<std::size_t>(std::floor(x / resolution()));
    const auto y_index = static_cast<std::size_t>(std::floor(y / resolution()));
    if (x_index >= width() || y_index >= height()) {
      return size();  // If the point is outside the map, return an invalid index
    }
    return x_index + y_index * width();
  }

  [[nodiscard]] std::size_t index(const Eigen::Vector2d& point) const { return index(point.x(), point.y()); }

  [[nodiscard]] Eigen::Vector2d point(std::size_t index) const {
    return Eigen::Vector2d{
        (static_cast<double>(index % width()) + 0.5) * resolution(),
        (static_cast<double>(index / width()) + 0.5) * resolution()};  // NOLINT(bugprone-integer-division)
  }

  [[nodiscard]] auto neighbors(std::size_t index) const {
    auto result = std::vector<std::size_t>{};
    const std::size_t row = index / width();
    const std::size_t col = index % width();
    if (row < (height() - 1)) {
      result.push_back(index + width());
    }
    if (row > 0) {
      result.push_back(index - width());
    }
    if (col < (width() - 1)) {
      result.push_back(index + 1);
    }
    if (col > 0) {
      result.push_back(index - 1);
    }
    return result;
  }
  [[nodiscard]] double resolution() const { return resolution_; }

 private:
  std::array<bool, Rows * Cols> grid_;
  Sophus::SE2d origin_;
  Sophus::SE2d origin_inverse_;
  double resolution_;

  [[nodiscard]] std::size_t width() const { return Cols; }
  [[nodiscard]] std::size_t height() const { return Rows; }
};

using UUT = ciabatta::mixin<
    ciabatta::curry<beluga::BeamSensorModel, StaticOccupancyGrid<5, 5>>::mixin,
    ciabatta::provides<beluga::LaserSensorModelInterface2d>::mixin>;

BeamModelParams GetParams(){
    BeamModelParams ret;
    ret.z_hit = 0.5;
    ret.z_rand = 0.5;
    ret.z_max = 0.05;
    ret.z_short = 0.05;
    ret.sigma_hit = 0.2;
    ret.lambda_short = 0.1;
    ret.max_beams = 60;
    ret.laser_max_range = 60;
    return ret;
}
TEST(BeamSensorModel, ImportanceWeight) {
  constexpr double kResolution = 0.5;
  // clang-format off
  const auto grid = StaticOccupancyGrid<5, 5>{{
    false, false, false, false, false,
    false, false, false, false, false,
    false, false, true , false, false,
    false, false, false, false, false,
    false, false, false, false, false},
    kResolution};
  // clang-format on

  const auto params = GetParams();
  auto mixin = UUT{params, grid};

  // Perfect hit.
  mixin.update_sensor(std::vector<std::pair<double, double>>{{1., 1.}});
  EXPECT_NEAR(0.13135474537037034, mixin.importance_weight(grid.origin()), 1e-6);

  // This is a hit that's before the obstacle, hence is affected by the unexpected obstacle part of the distribution.
  mixin.update_sensor(std::vector<std::pair<double, double>>{{0.6, 0.6}});
  EXPECT_NEAR(1.0771005220646571e-05, mixin.importance_weight(grid.origin()), 1e-6);

  // Hit that's past the obstacle, hence is not affected by the unexpected obstacle part of the distribution.
  // This should be really close to zero.
  mixin.update_sensor(std::vector<std::pair<double, double>>{{2.25, 2.25}});
  EXPECT_NEAR(0.000, mixin.importance_weight(grid.origin()), 1e-6);


  // Range return longer than laser_max_dist, so the random measurement distribution kicks in and this shouldn't be zero.
  mixin.update_sensor(std::vector<std::pair<double, double>>{{params.laser_max_range, params.laser_max_range}});
  EXPECT_NEAR(0.00012500000000000003, mixin.importance_weight(grid.origin()), 1e-6);


    }
    } // namespace beluga