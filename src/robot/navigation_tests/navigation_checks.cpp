#include "costmap_core.hpp"
#include "map_memory_core.hpp"
#include "planner_core.hpp"
#include "control_core.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
int checks = 0;
void check(bool value, const char* description) {
  if (!value) throw std::runtime_error(description);
  ++checks; std::cout << "PASS: " << description << '\n';
}
geometry_msgs::msg::Point point(double x, double y) {
  geometry_msgs::msg::Point p; p.x=x; p.y=y; return p;
}
int at(const nav_msgs::msg::OccupancyGrid& m, double x, double y) {
  int col = std::floor((x-m.info.origin.position.x)/m.info.resolution);
  int row = std::floor((y-m.info.origin.position.y)/m.info.resolution);
  return m.data.at(row*m.info.width+col);
}
int main() {
  try {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id="laser"; scan.range_min=0.1; scan.range_max=20;
    scan.angle_min=0; scan.angle_increment=1; scan.ranges={5};
    robot::CostmapCore perception(0.2, 21, 1, 1.5);
    auto local=perception.compute(scan);
    check(at(local, 5, 0)==100, "laser return is occupied");
    check(at(local, 2, 0)==0, "beam marks observed free space");
    check(at(local, 8, 0)==-1, "space behind obstacle remains unknown");
    check(at(local, 4.4, 0)==100, "robot clearance blocks nearby cells");
    scan.ranges={std::numeric_limits<float>::quiet_NaN()};
    auto invalid=perception.compute(scan);
    check(at(invalid, 0, 0)==-1, "NaN does not invent observations");
    scan.ranges={std::numeric_limits<float>::infinity()};
    check(at(perception.compute(scan), 5, 0)==0, "positive infinity clears beam up to maximum range");
    robot::MapMemoryCore memory(0.25, 20);
    memory.integrate(local, 2, 3, std::acos(-1.0)/2);
    check(at(memory.map(), 2, 8)==100, "90-degree rotation and translation place obstacle correctly");
    memory.integrate(invalid, 2, 3, 0);
    check(at(memory.map(), 2, 8)==100, "unknown observations preserve memory");
    memory.integrate(local, 1000, 1000, 0);
    check(at(memory.map(), 2, 8)==100, "out-of-bounds observations are ignored");
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id="sim_world"; map.info.resolution=1;
    map.info.width=map.info.height=10; map.info.origin.orientation.w=1;
    map.data.assign(100,0);
    for (int y=0;y<8;++y) map.data[y*10+4]=100;
    robot::PlannerCore planner;
    auto path=planner.plan(map,point(1.5,1.5),point(7.5,1.5));
    check(!path.poses.empty(), "A* finds a detour around a wall");
    bool avoids=true, reaches_gap=false;
    for (auto& p:path.poses) { avoids &= at(map,p.pose.position.x,p.pose.position.y)<100; reaches_gap |= p.pose.position.y>=8; }
    check(avoids && reaches_gap, "detour uses free cells through the actual opening");
    map.data[8*10+4]=map.data[9*10+4]=100;
    check(planner.plan(map,point(1.5,1.5),point(7.5,1.5)).poses.empty(), "sealed wall produces no path");
    check(planner.plan(map,point(-1,1),point(7,1)).poses.empty(), "outside start is rejected");
    check(planner.plan(map,point(1,1),point(4,1)).poses.empty(), "occupied goal is rejected");
    map.info.width=map.info.height=2; map.data={0,100,100,0};
    check(planner.plan(map,point(0.5,0.5),point(1.5,1.5)).poses.empty(), "diagonal cannot cut between two blocked cells");
    map.data={0,-1,-1,0};
    check(!planner.plan(map,point(0.5,0.5),point(1.5,1.5)).poses.empty(), "unknown space is traversable under the documented policy");
    robot::ControlCore controller;
    geometry_msgs::msg::Pose pose; pose.orientation.w=1;
    nav_msgs::msg::Path straight;
    geometry_msgs::msg::PoseStamped waypoint; waypoint.pose.position=point(3,0); straight.poses.push_back(waypoint);
    auto cmd=controller.command(straight,pose);
    check(cmd.linear.x>0 && std::abs(cmd.angular.z)<1e-9, "straight path commands forward motion");
    straight.poses[0].pose.position=point(0,3);
    cmd=controller.command(straight,pose);
    check(cmd.linear.x==0 && cmd.angular.z>0, "target on left turns left before driving");
    straight.poses[0].pose.position=point(0.1,0);
    cmd=controller.command(straight,pose);
    check(cmd.linear.x==0 && cmd.angular.z==0, "arrival commands a complete stop");
    cmd=controller.command(nav_msgs::msg::Path(),pose);
    check(cmd.linear.x==0 && cmd.angular.z==0, "empty path commands a complete stop");
    std::cout << checks << " behavioural checks passed\n";
  } catch(const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
