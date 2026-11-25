#include <beluga_visual_odometry_ros/visual_odometry_node.hpp>
#include <cv_bridge/cv_bridge.h>

namespace beluga_visual_odometry_ros {
VisualOdometryNode::VisualOdometryNode(const rclcpp::NodeOptions& options)
    : Node("visual_odometry_node", options),
      left_sub_(this, "left/image_raw"),
      right_sub_(this, "right/image_raw"),
      sync_(SyncPolicy(10), left_sub_, right_sub_) {
  beluga_visual_odometry::VisualOdometryConfig config;
  vo_ = std::make_unique<beluga_visual_odometry::StereoVisualOdometry>(config);
  sync_.registerCallback(&VisualOdometryNode::ImageCallback, this);
  pub_ = this->create_publisher<beluga_visual_odometry_ros::msg::VisualOdometry>("vo_output", 10);
}

void VisualOdometryNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                                       const sensor_msgs::msg::Image::ConstSharedPtr& right) {
  auto left_cv = cv_bridge::toCvShare(left, "mono8")->image;
  auto right_cv = cv_bridge::toCvShare(right, "mono8")->image;
  auto motion = vo_->Process(left_cv, right_cv, left->header.stamp.sec);
  beluga_visual_odometry_ros::msg::VisualOdometry msg;
  msg.delta.x = motion.delta[0];
  msg.delta.y = motion.delta[1];
  msg.delta.z = motion.delta[2];
  pub_->publish(msg);
}
}