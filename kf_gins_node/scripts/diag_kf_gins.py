#!/usr/bin/env python3
# diag_kf_gins.py
# Usage:
#   chmod +x diag_kf_gins.py
#   ros2 run <your_package> diag_kf_gins.py
# or run directly with: python3 diag_kf_gins.py (needs rclpy installed and sourced)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import math
import time

def quat_norm(q):
    return math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)

def quat_to_euler_deg(q):
    # convert quaternion (x,y,z,w) to roll, pitch, yaw in degrees
    x,y,z,w = q.x, q.y, q.z, q.w
    # roll (x-axis rotation)
    sinr_cosp = 2.0*(w*x + y*z)
    cosr_cosp = 1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis)
    sinp = 2.0*(w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw (z-axis)
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class DiagNode(Node):
    def __init__(self):
        super().__init__('diag_kf_gins')
        self.create_subscription(Odometry, '/kf_gins/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/kf_gins/path', self.path_cb, 10)
        self.last_path_len = 0
        self.last_path_z = None
        self.last_time = time.time()

    def path_cb(self, msg: Path):
        self.last_path_len = len(msg.poses)
        if self.last_path_len > 0:
            self.last_path_z = msg.poses[-1].pose.position.z

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        n = quat_norm(q)
        r,pitch,yaw = quat_to_euler_deg(q)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        print("---- odom @ %.9f (dt=%.3fs) ----" % (t, dt))
        print("pos: x=%.6f y=%.6f z=%.6f" % (p.x, p.y, p.z))
        print("quat: x=%.6f y=%.6f z=%.6f w=%.6f  |  norm=%.6f" % (q.x, q.y, q.z, q.w, n))
        print("euler(deg): roll=%.3f pitch=%.3f yaw=%.3f" % (r, pitch, yaw))
        print("path_len=%d last_path_z=%s" % (self.last_path_len, ("%.6f" % self.last_path_z) if self.last_path_z is not None else "None"))
        print(" ")

def main(args=None):
    rclpy.init(args=args)
    node = DiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
