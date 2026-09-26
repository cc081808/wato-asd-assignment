#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
namespace robot {
// This class does the mathematics. The node handles ROS communication.
class CostmapCore {
 public:
  CostmapCore(double resolution = 0.2, double extent = 21.0,
              double clearance = 2.0, double inflation = 2.6);
  nav_msgs::msg::OccupancyGrid compute(const sensor_msgs::msg::LaserScan& scan) const;
 private:
  double resolution_, extent_, clearance_, inflation_;
};
}
