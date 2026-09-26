#!/usr/bin/env python3
"""Verify arrival stop and no motion for an unreachable goal in the live simulator."""
import json
import math
import time
import rclpy
from geometry_msgs.msg import PointStamped
from check_navigation import Probe

rclpy.init()
n = Probe()
ready = n.wait(lambda: n.odom is not None and n.cmd is not None, 15)
n.wait(lambda: False, 2)
arrival = ready and abs(n.cmd.linear.x) < 1e-6 and abs(n.cmd.angular.z) < 1e-6
start = n.odom.pose.pose.position if n.odom else None
goal = PointStamped()
goal.header.frame_id = 'sim_world'
goal.point.x = 0.0
goal.point.y = 0.0  # Inside the central cylinder: no route should exist.
n.pub.publish(goal)
n.wait(lambda: False, 2)
observed = []
end = time.monotonic() + 3
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.cmd:
        observed.append([n.cmd.linear.x, n.cmd.angular.z])
p = n.odom.pose.pose.position if n.odom else None
drift = math.hypot(p.x-start.x, p.y-start.y) if p and start else None
result = {'arrival_stop': arrival, 'unreachable_goal': [0,0],
          'zero_commands': bool(observed) and all(abs(v)<1e-6 and abs(w)<1e-6 for v,w in observed),
          'position_drift_m': drift, 'command_observations': len(observed)}
print(json.dumps(result, indent=2))
n.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if arrival and result['zero_commands'] and drift is not None and drift < 0.02 else 1)
