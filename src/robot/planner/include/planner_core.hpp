#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
namespace robot {
class PlannerCore {
 public:
  nav_msgs::msg::Path plan(const nav_msgs::msg::OccupancyGrid& map,
                          const geometry_msgs::msg::Point& start,
                          const geometry_msgs::msg::Point& goal) const;
};
}
