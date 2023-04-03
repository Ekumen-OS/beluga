#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <iostream>

namespace beluga{

  /// Leverages a Brasenhaum raycasting technique to cast a ray on the occupancy grid, \n
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
  template <class OccupancyGrid>
  inline double raycast(const OccupancyGrid& grid, const Sophus::SE2d& starting_position_in_map_frame,
   const Sophus::SO2d& bearing_in_laser_frame,
    double max_beam_range){

    const auto laser_position_in_grid_frame = grid.origin().inverse() * starting_position_in_map_frame;

    auto is_not_free = [&](const auto val){
      return !OccupancyGrid::Traits::is_free(val);
    };

    auto grid_at = [&grid](std::size_t index){
      return grid.data()[index];
    };

    const auto bearing_in_grid_frame = Sophus::SE2d{laser_position_in_grid_frame.so2() * bearing_in_laser_frame, Eigen::Vector2d::Zero()};
    const auto ray_in_grid_frame = bearing_in_grid_frame * Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{max_beam_range, 0}};


    auto double_to_int_grid_coords = [&](const Eigen::Vector2d& point) -> Eigen::Vector2i{
      return Eigen::Vector2d{(point / grid.resolution()).array().floor()}.cast<int>();
    };
    const auto pt_0 = double_to_int_grid_coords(laser_position_in_grid_frame.translation());
    const auto pt_1 = double_to_int_grid_coords((laser_position_in_grid_frame * ray_in_grid_frame).translation());

    std::cerr << " Point 0 : " << pt_0 << std::endl;
    std::cerr << " Point 1 : " << pt_1 << std::endl;

    int x0 = pt_0.x();
    int y0 = pt_0.y();

    int x1 = pt_1.x();
    int y1 = pt_1.y();

    int delta_x = std::abs(x1 - x0);
    int delta_y = std::abs(y1 - y0);

    const bool steep = delta_y > delta_x;
    if (steep) {
      std::cerr << "Steep" << std::endl;
      std::swap(x0, y0);
      std::swap(x1, y1);
      delta_x = std::abs(x1 - x0);
      delta_y = std::abs(y1 - y0);
    }


    int x = x0;
    int y = y0;

    const int x_step = x0 < x1 ? 1 : -1;
    const int y_step = y0 < y1 ? 1 : -1;

    const auto center_of_first_cell = grid.point(grid.index(x0 * grid.resolution(), y0 * grid.resolution()));
    int error = 0;
    while (x != (x1 + x_step)) {
      const std::size_t index = steep ? grid.index(y * grid.resolution(),x * grid.resolution()) : grid.index(x * grid.resolution() ,y * grid.resolution());
      auto pt = grid.point(index);
      std::cerr << "Testing " << pt.x() << " , " << pt.y() << std::endl;

      if (index == grid.size() || is_not_free(grid_at(index))) {
        std::cerr << "here 3" << std::endl;
        return (grid.point(index) - center_of_first_cell).norm();
      }
      x += 1;
      error += delta_y;
      if (2 * error >= delta_x) {
        std::cerr << "Updating Y " << std::endl;
        y += y_step;
        error -= delta_x;
      }
    }
    return max_beam_range;
  }

}
