#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class FixCov(Node):
    def __init__(self):
        super().__init__('fix_cov_repub')
        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix_cov', 10)

    def cb(self, msg: NavSatFix):
        # 设定 ~3 m 的水平、5 m 的高程（方差=σ²）
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        msg.position_covariance = [9.0, 0.0, 0.0,
                                   0.0, 9.0, 0.0,
                                   0.0, 0.0, 25.0]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FixCov()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
