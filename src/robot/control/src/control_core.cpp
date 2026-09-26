#include "control_core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace robot {
ControlCore::ControlCore(double speed, double lookahead, double tolerance)
    : speed_(speed), lookahead_(lookahead), tolerance_(tolerance) {
  if (!(speed > 0 && lookahead > 0 && tolerance > 0)) throw std::invalid_argument("Invalid control parameters");
}
geometry_msgs::msg::Twist ControlCore::command(const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Pose& pose) const {
  geometry_msgs::msg::Twist cmd;
  if (path.poses.empty()) return cmd;
  auto distance = [&](const geometry_msgs::msg::Point& p) { return std::hypot(p.x-pose.position.x, p.y-pose.position.y); };
  const double remaining = distance(path.poses.back().pose.position);
  if (remaining <= tolerance_) return cmd;
  // Find the nearest waypoint, then walk forward along the route.
  size_t nearest = 0;
  for (size_t i = 1; i < path.poses.size(); ++i)
    if (distance(path.poses[i].pose.position) < distance(path.poses[nearest].pose.position)) nearest = i;
  size_t target = nearest;
  while (target + 1 < path.poses.size() && distance(path.poses[target].pose.position) < lookahead_) ++target;
  const auto& p = path.poses[target].pose.position;
  const auto& q = pose.orientation;
  const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));
  const double dx = p.x-pose.position.x, dy = p.y-pose.position.y;
  const double x = std::cos(yaw)*dx + std::sin(yaw)*dy;
  const double y = -std::sin(yaw)*dx + std::cos(yaw)*dy;
  const double angle = std::atan2(y, x);
  if (std::abs(angle) > 0.65) { // Face a new route before driving forward.
    cmd.angular.z = std::clamp(1.2 * angle, -0.6, 0.6);
    return cmd;
  }
  cmd.linear.x = std::min(speed_, remaining * 0.7) * std::max(0.2, std::cos(angle));
  cmd.angular.z = std::clamp(2 * cmd.linear.x * y / std::max(0.01, x*x+y*y), -0.6, 0.6);
  return cmd;
}
}
