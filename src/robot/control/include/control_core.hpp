#pragma once
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
namespace robot {
class ControlCore {
 public:
  ControlCore(double speed = 0.6, double lookahead = 0.65, double tolerance = 0.35);
  geometry_msgs::msg::Twist command(const nav_msgs::msg::Path& path,
                                   const geometry_msgs::msg::Pose& pose) const;
 private:
  double speed_, lookahead_, tolerance_;
};
}
