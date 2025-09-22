#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

class ImuEcho(Node):
    def __init__(self):
        super().__init__('imu_echo_node')
        # 订阅IMU话题（使用传感器QoS）
        self.sub = self.create_subscription(
            Imu, '/imu/data', self.callback, 10  # QoS深度设为10
        )
        self.last_print_time = 0.0
        self.print_interval = 1.0  # 打印间隔（秒），控制频率

    def callback(self, msg: Imu):
        current_time = time.time()
        # 每隔指定时间打印一次
        if current_time - self.last_print_time >= self.print_interval:
            self.last_print_time = current_time
            # 打印角速度和线加速度
            self.get_logger().info(f"IMU数据:")
            self.get_logger().info(f"  角速度: x={msg.angular_velocity.x:.6f}, y={msg.angular_velocity.y:.6f}, z={msg.angular_velocity.z:.6f}")
            self.get_logger().info(f"  线加速度: x={msg.linear_acceleration.x:.6f}, y={msg.linear_acceleration.y:.6f}, z={msg.linear_acceleration.z:.6f}")

def main():
    rclpy.init()
    node = ImuEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("退出程序")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()