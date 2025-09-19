#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math, time

class Checker(Node):
    def __init__(self):
        super().__init__('imu_bad_checker')
        self.sub = self.create_subscription(Imu, '/imu/data', self.cb, 10)
        self.count = 0
        self.bad = 0
        self.start = time.time()

    def cb(self, msg):
        self.count += 1
        def finite(x): return (not math.isinf(x)) and (not math.isnan(x))
        ok = True
        # stamp sanity
        try:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            if not math.isfinite(ts): ok = False
        except:
            ok = False
        arr = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ]
        for v in arr:
            if not finite(v):
                ok = False
        # check extremes
        if ok:
            if any(abs(v) > 1e3 for v in arr): ok = False
        if not ok:
            self.bad += 1
            self.get_logger().warn(f'BAD IMU sample #{self.count}: stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec} values={arr}')
        else:
            if self.count % 200 == 0:
                self.get_logger().info(f'ok samples={self.count} bad={self.bad}')
        if self.count >= 2000 or (time.time() - self.start) > 20.0:
            self.get_logger().info(f'finished: total={self.count} bad={self.bad}')
            rclpy.shutdown()

    def main():
        rclpy.init()
        node = Checker()
        rclpy.spin(node)

    if __name__ == '__main__':
        main()
