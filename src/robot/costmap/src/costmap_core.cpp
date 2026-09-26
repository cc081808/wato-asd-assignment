#include "costmap_core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace robot {
CostmapCore::CostmapCore(double resolution, double extent, double clearance, double inflation)
    : resolution_(resolution), extent_(extent), clearance_(clearance), inflation_(inflation) {
  if (!(resolution > 0 && extent > clearance && clearance > 0 && inflation >= clearance))
    throw std::invalid_argument("Invalid costmap geometry");
}
nav_msgs::msg::OccupancyGrid CostmapCore::compute(const sensor_msgs::msg::LaserScan& scan) const {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = scan.header;
  grid.info.resolution = resolution_;
  grid.info.width = grid.info.height = static_cast<unsigned>(std::ceil(2 * extent_ / resolution_));
  grid.info.origin.position.x = grid.info.origin.position.y = -extent_;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(grid.info.width * grid.info.height, -1); // -1 means not observed.
  const int n = static_cast<int>(grid.info.width);
  // ROS stores resolution as float. Use that exact stored value for indexing too.
  const double cell_size = grid.info.resolution;
  auto cell = [&](double x) { return static_cast<int>(std::floor((x + extent_) / cell_size)); };
  auto inside = [&](int x, int y) { return x >= 0 && y >= 0 && x < n && y < n; };
  std::vector<std::pair<int, int>> hits;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (std::isnan(range) || range < scan.range_min || !(scan.range_max > 0)) continue;
    const bool hit = std::isfinite(range) && range < scan.range_max;
    const double distance = std::min(hit ? range : double(scan.range_max), extent_ - resolution_);
    const double angle = scan.angle_min + i * scan.angle_increment;
    const double c = std::cos(angle), s = std::sin(angle);
    // Mark only cells traversed by a beam as free. Space behind a return stays unknown.
    for (double d = 0; d < distance; d += resolution_ * 0.5) {
      const int x = cell(d * c), y = cell(d * s);
      if (inside(x, y)) grid.data[y * n + x] = 0;
    }
    if (hit && range <= distance) {
      const int x = cell(range * c), y = cell(range * s);
      if (inside(x, y)) hits.emplace_back(x, y);
    }
  }
  // Inflate after ray tracing so another beam cannot erase an obstacle's clearance.
  const int radius = static_cast<int>(std::ceil(inflation_ / resolution_));
  for (const auto& hit : hits) {
    for (int dy = -radius; dy <= radius; ++dy) for (int dx = -radius; dx <= radius; ++dx) {
      const int x = hit.first + dx, y = hit.second + dy;
      const double d = std::hypot(dx, dy) * resolution_;
      if (!inside(x, y) || d > inflation_) continue;
      const int value = d <= clearance_ ? 100 :
        std::max(1, static_cast<int>(98 * (inflation_ - d) / (inflation_ - clearance_)));
      auto& stored = grid.data[y * n + x];
      stored = static_cast<int8_t>(std::max<int>(stored, value));
    }
  }
  return grid;
}
}
