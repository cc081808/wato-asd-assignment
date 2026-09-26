#include "costmap_node.hpp"
CostmapNode::CostmapNode() : Node("costmap_node"), core_(
    declare_parameter("resolution", 0.2), declare_parameter("extent", 21.0),
    declare_parameter("clearance", 2.0), declare_parameter("inflation", 2.6)) {
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 1);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/lidar", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) { map_pub_->publish(core_.compute(*scan)); });
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
}
