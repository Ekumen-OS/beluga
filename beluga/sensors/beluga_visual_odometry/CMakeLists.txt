cmake_minimum_required(VERSION 3.8)
project(beluga_visual_odometry)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/motion_estimator.cpp
  src/planar_projection.cpp
  src/stereo_calibration.cpp
  src/stereo_frontend.cpp
  src/stereo_visual_odometry.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  Sophus::Sophus
  OpenCV::OpenCV
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  cv_bridge::cv_bridge
)

ament_package()