#include "map_memory_core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace robot {
MapMemoryCore::MapMemoryCore(double resolution, double extent) {
  if (!(resolution > 0 && extent > 0)) throw std::invalid_argument("Invalid global map geometry");
  map_.header.frame_id = "sim_world";
  map_.info.resolution = resolution;
  map_.info.width = map_.info.height = static_cast<unsigned>(std::ceil(2 * extent / resolution));
  map_.info.origin.position.x = map_.info.origin.position.y = -extent;
  map_.info.origin.orientation.w = 1;
  map_.data.assign(map_.info.width * map_.info.height, -1);
}
void MapMemoryCore::integrate(const nav_msgs::msg::OccupancyGrid& local, double x, double y, double yaw) {
  if (local.data.size() != size_t(local.info.width) * local.info.height) return;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (unsigned row = 0; row < local.info.height; ++row) for (unsigned col = 0; col < local.info.width; ++col) {
    const int value = local.data[row * local.info.width + col];
    if (value < 0) continue;
    const double lx = local.info.origin.position.x + (col + 0.5) * local.info.resolution;
    const double ly = local.info.origin.position.y + (row + 0.5) * local.info.resolution;
    const int gx = static_cast<int>(std::floor((x + c * lx - s * ly - map_.info.origin.position.x) / map_.info.resolution));
    const int gy = static_cast<int>(std::floor((y + s * lx + c * ly - map_.info.origin.position.y) / map_.info.resolution));
    if (gx < 0 || gy < 0 || gx >= int(map_.info.width) || gy >= int(map_.info.height)) continue;
    auto& stored = map_.data[gy * map_.info.width + gx];
    // Conservative memory for STATIC objects. See docs/DECISIONS.md for the tradeoff.
    stored = static_cast<int8_t>(std::max<int>(stored, value));
  }
  map_.header.stamp = local.header.stamp;
}
}
