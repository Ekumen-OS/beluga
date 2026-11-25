cmake_minimum_required(VERSION 3.8)
project(beluga_visual_odometry_ros)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(beluga_visual_odometry REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/VisualOdometry.msg")

add_library(${PROJECT_NAME} SHARED src/visual_odometry_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  cv_bridge::cv_bridge
  message_filters::message_filters
  beluga_visual_odometry::beluga_visual_odometry
  ${PROJECT_NAME}_generate_py
)

ament_package()