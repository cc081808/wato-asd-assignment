#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "map_memory_core.hpp"
class MapMemoryNode : public rclcpp::Node {
 public:
  MapMemoryNode();
 private:
  void update();
  robot::MapMemoryCore core_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr local_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
