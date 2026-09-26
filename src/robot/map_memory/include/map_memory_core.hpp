#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>
namespace robot {
class MapMemoryCore {
 public:
  MapMemoryCore(double resolution = 0.25, double extent = 20.0);
  void integrate(const nav_msgs::msg::OccupancyGrid& local, double x, double y, double yaw);
  const nav_msgs::msg::OccupancyGrid& map() const { return map_; }
 private:
  nav_msgs::msg::OccupancyGrid map_;
};
}
