#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "control_core.hpp"
class ControlNode : public rclcpp::Node {
 public:
  ControlNode();
 private:
  void update();
  robot::ControlCore core_;
  nav_msgs::msg::Path::ConstSharedPtr path_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  std::chrono::steady_clock::time_point path_time_, odom_time_, scan_time_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
