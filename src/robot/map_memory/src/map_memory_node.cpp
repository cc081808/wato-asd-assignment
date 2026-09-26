#include "map_memory_node.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
MapMemoryNode::MapMemoryNode() : Node("map_memory_node"), core_(
    declare_parameter("resolution", 0.25), declare_parameter("extent", 20.0)),
    buffer_(get_clock()), listener_(buffer_) {
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 1,
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { local_ = msg; });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 1,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_ = msg; });
  map_pub_->publish(core_.map()); // Avoid the startup deadlock described in the assignment.
  timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() { update(); });
}
void MapMemoryNode::update() {
  if (!local_ || !odom_) return;
  try {
    // Use the pose at scan time, not the latest pose: the robot may have turned since then.
    const auto tf = buffer_.lookupTransform("sim_world", local_->header.frame_id,
                                            rclcpp::Time(local_->header.stamp));
    core_.integrate(*local_, tf.transform.translation.x, tf.transform.translation.y,
                    tf2::getYaw(tf.transform.rotation));
    map_pub_->publish(core_.map());
    local_.reset();
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for scan transform: %s", ex.what());
  }
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
}
