#include "control_node.hpp"
#include <cmath>
ControlNode::ControlNode() : Node("control_node"), core_(declare_parameter("speed", 0.6),
    declare_parameter("lookahead", 0.65), declare_parameter("goal_tolerance", 0.35)) {
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  path_sub_ = create_subscription<nav_msgs::msg::Path>("/path", 1,
    [this](nav_msgs::msg::Path::ConstSharedPtr msg) { path_ = msg; path_time_ = std::chrono::steady_clock::now(); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 1,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; odom_time_ = std::chrono::steady_clock::now(); });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/lidar", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { scan_ = msg; scan_time_ = std::chrono::steady_clock::now(); });
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { update(); });
}
void ControlNode::update() {
  geometry_msgs::msg::Twist cmd;
  const auto time = std::chrono::steady_clock::now();
  auto fresh = [&](auto t) { return std::chrono::duration<double>(time - t).count() < 1.5; };
  if (path_ && odom_ && scan_ && fresh(path_time_) && fresh(odom_time_) && fresh(scan_time_) &&
      path_->header.frame_id == odom_->header.frame_id) {
    cmd = core_.command(*path_, odom_->pose.pose);
    // A final short-range guard protects against newly seen obstacles before replanning.
    if (cmd.linear.x > 0) for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double r = scan_->ranges[i], a = scan_->angle_min + i * scan_->angle_increment;
      if (std::isfinite(r) && r >= scan_->range_min && r * std::cos(a) > 0 &&
          r * std::cos(a) < 1.2 && std::abs(r * std::sin(a)) < 0.85) { cmd = geometry_msgs::msg::Twist(); break; }
    }
  }
  cmd_pub_->publish(cmd);
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<ControlNode>()); rclcpp::shutdown();
}
