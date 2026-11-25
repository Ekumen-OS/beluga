#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <beluga_visual_odometry/stereo_visual_odometry.hpp>
#include <beluga_visual_odometry_ros/msg/visual_odometry.hpp>

namespace beluga_visual_odometry_ros {
class VisualOdometryNode : public rclcpp::Node {
public:
  explicit VisualOdometryNode(const rclcpp::NodeOptions& options);
private:
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                     const sensor_msgs::msg::Image::ConstSharedPtr& right);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_, right_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  rclcpp::Publisher<beluga_visual_odometry_ros::msg::VisualOdometry>::SharedPtr pub_;
  std::unique_ptr<beluga_visual_odometry::StereoVisualOdometry> vo_;
};
}