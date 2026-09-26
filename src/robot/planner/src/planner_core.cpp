#include "planner_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
namespace robot {
nav_msgs::msg::Path PlannerCore::plan(const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& goal) const {
  nav_msgs::msg::Path path;
  path.header = map.header;
  const int w = map.info.width, h = map.info.height;
  const double resolution = map.info.resolution;
  if (w <= 0 || h <= 0 || resolution <= 0 || map.data.size() != size_t(w) * h) return path;
  auto index = [&](const geometry_msgs::msg::Point& p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return -1;
    const double x = std::floor((p.x - map.info.origin.position.x) / resolution);
    const double y = std::floor((p.y - map.info.origin.position.y) / resolution);
    return x < 0 || y < 0 || x >= w || y >= h ? -1 : int(y) * w + int(x);
  };
  const int source = index(start), target = index(goal);
  if (source < 0 || target < 0 || map.data[source] >= 100 || map.data[target] >= 100) return path;
  auto heuristic = [&](int id) { return std::hypot(id % w - target % w, id / w - target / w); };
  auto free = [&](int x, int y) { return x >= 0 && y >= 0 && x < w && y < h && map.data[y*w+x] < 100; };
  std::vector<double> distance(w * h, std::numeric_limits<double>::infinity());
  std::vector<int> parent(w * h, -1);
  std::vector<bool> closed(w * h, false);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  distance[source] = 0;
  open.emplace(heuristic(source), source);
  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[current]) continue;
    if (current == target) break;
    closed[current] = true;
    const int x = current % w, y = current / w;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
      if ((dx == 0 && dy == 0) || !free(x + dx, y + dy)) continue;
      if (dx != 0 && dy != 0 && (!free(x + dx, y) || !free(x, y + dy))) continue;
      const int next = (y + dy) * w + x + dx;
      const int cost = map.data[next];
      // Unknown is traversable with a penalty; later observations trigger replanning.
      const double penalty = cost < 0 ? 2.0 : 1.0 + 3.0 * cost / 100.0;
      const double candidate = distance[current] + std::hypot(dx, dy) * penalty;
      if (candidate < distance[next]) {
        distance[next] = candidate;
        parent[next] = current;
        open.emplace(candidate + heuristic(next), next);
      }
    }
  }
  if (!std::isfinite(distance[target])) return path;
  for (int at = target; at != -1; at = parent[at]) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = map.header;
    pose.pose.position.x = map.info.origin.position.x + (at % w + 0.5) * resolution;
    pose.pose.position.y = map.info.origin.position.y + (at / w + 0.5) * resolution;
    pose.pose.orientation.w = 1;
    path.poses.push_back(pose);
  }
  std::reverse(path.poses.begin(), path.poses.end());
  path.poses.back().pose.position = goal;
  return path;
}
}
