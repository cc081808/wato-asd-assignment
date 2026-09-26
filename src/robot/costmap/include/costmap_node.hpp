#pragma once
#include <rclcpp/rclcpp.hpp>
#include "costmap_core.hpp"
class CostmapNode : public rclcpp::Node {
 public:
  CostmapNode();
 private:
  robot::CostmapCore core_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};
