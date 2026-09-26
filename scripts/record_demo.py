#!/usr/bin/env python3
"""Record one real local navigation trip for offline Foxglove inspection."""
import json
import math
import os
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import PointStamped
from check_navigation import Probe

destination='/tmp/asd-learning-demo'
recorder=subprocess.Popen(['ros2','bag','record','-o',destination,
    '/lidar','/costmap','/map','/path','/odom/filtered','/cmd_vel','/tf','/tf_static','/goal_point'],
    start_new_session=True)
rclpy.init();n=Probe();reached=False
try:
    ready=n.wait(lambda:n.odom is not None and n.map is not None,15)
    n.wait(lambda:False,3)
    if ready:
        goal=PointStamped();goal.header.frame_id='sim_world';goal.point.x=-10.;goal.point.y=-11.
        n.pub.publish(goal)
        reached=n.wait(lambda:math.hypot(n.odom.pose.pose.position.x+10,n.odom.pose.pose.position.y+11)<0.35,120)
        n.wait(lambda:False,2)
    p=n.odom.pose.pose.position if n.odom else None
    print(json.dumps({'goal':[-10,-11],'reached':reached,'final':None if p is None else [p.x,p.y],
                      'stopped':n.cmd is not None and abs(n.cmd.linear.x)<1e-6 and abs(n.cmd.angular.z)<1e-6}),flush=True)
finally:
    os.killpg(recorder.pid,signal.SIGINT);recorder.wait(timeout=10)
    n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if reached else 1)
