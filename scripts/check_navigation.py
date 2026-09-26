#!/usr/bin/env python3
"""Send goals to the isolated simulator and save REAL received telemetry as JSON.
Run inside the robot container after bringing up compose.learning.yaml.
"""
import argparse, json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan

class Probe(Node):
    def __init__(self):
        super().__init__('navigation_probe')
        self.odom = self.map = self.scan = self.cmd = self.path = None
        self.samples = []
        self.create_subscription(Odometry, '/odom/filtered', self.on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', lambda m:setattr(self,'map',m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/lidar', lambda m:setattr(self,'scan',m), qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', lambda m:setattr(self,'cmd',m), 10)
        self.create_subscription(Path, '/path', lambda m:setattr(self,'path',m), 10)
        self.pub = self.create_publisher(PointStamped, '/goal_point', 10)

    def on_odom(self, msg):
        self.odom=msg
        p=msg.pose.pose.position
        q=msg.pose.pose.orientation
        self.samples.append({'t':time.time(),'x':p.x,'y':p.y,
            'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
            'v': self.cmd.linear.x if self.cmd else 0,
            'w': self.cmd.angular.z if self.cmd else 0})

    def wait(self, predicate, timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=0.1)
            if predicate(): return True
        return False

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--goals', default='-7,10;10,10;10,-11')
    parser.add_argument('--timeout', type=float, default=150)
    parser.add_argument('--output', default='/tmp/navigation-run.json')
    args=parser.parse_args()
    rclpy.init(); n=Probe(); results=[]
    ready=n.wait(lambda:n.odom is not None and n.map is not None and n.scan is not None and any(v==100 for v in n.map.data),30)
    if ready:
        for item in args.goals.split(';'):
            x,y=map(float,item.split(','))
            goal=PointStamped(); goal.header.frame_id='sim_world'; goal.point.x=x; goal.point.y=y
            goal.header.stamp=n.get_clock().now().to_msg()
            n.pub.publish(goal)
            print(f'Goal ({x}, {y}) sent',flush=True)
            reached=n.wait(lambda:math.hypot(n.odom.pose.pose.position.x-x,n.odom.pose.pose.position.y-y)<0.4,args.timeout)
            p=n.odom.pose.pose.position
            result={'goal':[x,y],'reached':reached,'final':[p.x,p.y],'distance':math.hypot(p.x-x,p.y-y)}
            results.append(result); print(json.dumps(result),flush=True)
            if not reached: break
            n.wait(lambda:False,2)
    output={'ready':ready,'results':results,'samples':n.samples,
        'map':None if n.map is None else {'width':n.map.info.width,'height':n.map.info.height,
        'resolution':n.map.info.resolution,'origin':[n.map.info.origin.position.x,n.map.info.origin.position.y],
        'data':list(n.map.data)}}
    with open(args.output,'w') as f:json.dump(output,f)
    n.destroy_node();rclpy.shutdown()
    return 0 if ready and results and all(r['reached'] for r in results) else 1
if __name__=='__main__':raise SystemExit(main())
