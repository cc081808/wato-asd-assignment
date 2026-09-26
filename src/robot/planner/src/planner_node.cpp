#include "planner_node.hpp"
#include <cmath>
PlannerNode::PlannerNode() : Node("planner_node"), tolerance_(declare_parameter("goal_tolerance", 0.35)) {
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 1);
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { map_ = msg; });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 1,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; });
  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 1,
    [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
      if (msg->header.frame_id != "sim_world" || !std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) {
        RCLCPP_WARN(get_logger(), "Goal must be finite and in sim_world"); return;
      }
      goal_ = msg->point; goal_.z = 0; active_ = true; update();
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { update(); });
}
void PlannerNode::update() {
  nav_msgs::msg::Path path;
  path.header.frame_id = "sim_world";
  if (active_ && map_ && odom_ && map_->header.frame_id == odom_->header.frame_id) {
    const auto& p = odom_->pose.pose.position;
    if (std::hypot(p.x - goal_.x, p.y - goal_.y) <= tolerance_) {
      active_ = false;
      RCLCPP_INFO(get_logger(), "Goal reached");
    } else {
      path = core_.plan(*map_, p, goal_);
      if (path.poses.empty()) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No route: waiting stopped for a reachable goal/map");
    }
  }
  path.header.stamp = now();
  path_pub_->publish(path); // Empty paths explicitly cancel previous motion.
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<PlannerNode>()); rclcpp::shutdown();
}
