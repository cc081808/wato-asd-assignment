#!/usr/bin/env python3
"""Validate the beginner publisher by receiving a real ROS message."""
import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node=Node('warmup_probe')
messages=[]
subscription=node.create_subscription(String,'/test_topic',lambda m:messages.append(m.data),10)
process=subprocess.Popen(['ros2','run','navigation_tests','warmup_publisher'],start_new_session=True)
try:
    deadline=time.monotonic()+10
    while time.monotonic()<deadline and len(messages)<3:
        rclpy.spin_once(node,timeout_sec=0.2)
    passed=len(messages)>=3 and all(m=='Hello, ROS 2!' for m in messages)
    print(f'Warm-up received {len(messages)} messages: {messages}; PASS={passed}',flush=True)
finally:
    os.killpg(process.pid,signal.SIGINT)
    process.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if passed else 1)
