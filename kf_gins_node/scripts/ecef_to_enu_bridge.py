#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# 地理常数
a = 6378137.0               # WGS-84 椭球长半轴
f = 1/298.257223563
b = a*(1-f)
e2 = 1 - (b*b)/(a*a)

def lla_to_ecef(lat, lon, h):
    lat = math.radians(lat); lon = math.radians(lon)
    sinl = math.sin(lat); cosl = math.cos(lat)
    sinL = math.sin(lon); cosL = math.cos(lon)
    N = a / math.sqrt(1 - e2*sinl*sinl)
    x = (N + h)*cosl*cosL
    y = (N + h)*cosl*sinL
    z = (N*(1 - e2) + h)*sinl
    return np.array([x, y, z])

def R_ecef_to_enu(lat, lon):
    lat = math.radians(lat); lon = math.radians(lon)
    sinl = math.sin(lat); cosl = math.cos(lat)
    sinL = math.sin(lon); cosL = math.cos(lon)
    # 将 ECEF 的增量旋到 ENU
    R = np.array([[-sinL,            cosL,           0],
                  [-sinl*cosL, -sinl*sinL,     cosl],
                  [ cosl*cosL,  cosl*sinL,     sinl]])
    return R

def quat_multiply(q1, q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def rotm_to_quat(R):
    # 从 3x3 旋转矩阵得到四元数（xyzw）
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t+1.0)*2
        w = 0.25*s
        x = (R[2,1]-R[1,2])/s
        y = (R[0,2]-R[2,0])/s
        z = (R[1,0]-R[0,1])/s
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            x = 0.25*s
            y = (R[0,1]+R[1,0])/s
            z = (R[0,2]+R[2,0])/s
            w = (R[2,1]-R[1,2])/s
        elif i == 1:
            s = math.sqrt(1.0 - R[0,0] + R[1,1] - R[2,2]) * 2
            x = (R[0,1]+R[1,0])/s
            y = 0.25*s
            z = (R[1,2]+R[2,1])/s
            w = (R[0,2]-R[2,0])/s
        else:
            s = math.sqrt(1.0 - R[0,0] - R[1,1] + R[2,2]) * 2
            x = (R[0,2]+R[2,0])/s
            y = (R[1,2]+R[2,1])/s
            z = 0.25*s
            w = (R[1,0]-R[0,1])/s
    return np.array([x,y,z,w])

class Bridge(Node):
    def __init__(self):
        super().__init__('ecef_to_enu_bridge')

        # 用你日志里的原点（可以改成参数）
        self.lat0 = 30.4447873701
        self.lon0 = 114.471863205
        self.h0   = 20.899

        self.p0_ecef = lla_to_ecef(self.lat0, self.lon0, self.h0)
        self.R = R_ecef_to_enu(self.lat0, self.lon0)
        self.q_R = rotm_to_quat(self.R)

        self.odom_sub = self.create_subscription(
            Odometry, '/kf_gins/odom', self.cb_odom, 20)

        self.odom_pub = self.create_publisher(
            Odometry, '/kf_gins/odom_local', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def cb_odom(self, msg: Odometry):
        p_ecef = np.array([msg.pose.pose.position.x,
                           msg.pose.pose.position.y,
                           msg.pose.pose.position.z])
        dp = p_ecef - self.p0_ecef
        p_enu = self.R @ dp   # 3x1

        # 旋转：q_enu = q_R ⊗ q_ecef
        q_ecef = np.array([msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w])
        q_enu = quat_multiply(self.q_R, q_ecef)

        # 发布新的 Odom（map 为本地 ENU）
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'map'
        out.child_frame_id  = msg.child_frame_id
        out.pose.pose.position.x = float(p_enu[0])
        out.pose.pose.position.y = float(p_enu[1])
        out.pose.pose.position.z = float(p_enu[2])
        out.pose.pose.orientation.x = float(q_enu[0])
        out.pose.pose.orientation.y = float(q_enu[1])
        out.pose.pose.orientation.z = float(q_enu[2])
        out.pose.pose.orientation.w = float(q_enu[3])
        out.twist = msg.twist
        self.odom_pub.publish(out)

        # 也发一条 map->base_link 的 tf 方便 RViz 看
        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = out.child_frame_id
        tf.transform.translation.x = out.pose.pose.position.x
        tf.transform.translation.y = out.pose.pose.position.y
        tf.transform.translation.z = out.pose.pose.position.z
        tf.transform.rotation = out.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

def main():
    rclpy.init()
    node = Bridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
