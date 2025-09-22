#!/usr/bin/env python3
"""
imu_check.py

Subscribe to /imu/data and analyze incoming samples to:
  - estimate typical dt
  - estimate whether angular_velocity/linear_acceleration are:
      * increments (delta) or rates/acc (SI)
      * accelerations in 'g' units or in m/s^2

Usage:
  source install/setup.bash
  python3 scripts/imu_check.py --samples 300 --timeout 8.0
"""

import rclpy
from rclpy.node import Node
import math
import time
import argparse
import statistics

# QoS compat: try SensorDataQoS, else use qos_profile_sensor_data or build a QoSProfile
try:
    # some rclpy versions provide SensorDataQoS directly
    from rclpy.qos import SensorDataQoS as _SensorDataQoS  # type: ignore
    def get_sensor_qos():
        return _SensorDataQoS()
except Exception:
    try:
        # some versions expose a preset constant
        from rclpy.qos import qos_profile_sensor_data  # type: ignore
        def get_sensor_qos():
            return qos_profile_sensor_data
    except Exception:
        # fallback: construct equivalent QoSProfile manually
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # type: ignore
        def get_sensor_qos():
            q = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5)
            q.reliability = QoSReliabilityPolicy.BEST_EFFORT
            return q

from sensor_msgs.msg import Imu

class ImuChecker(Node):
    def __init__(self, samples=300, timeout=8.0):
        super().__init__('imu_checker')
        qos = get_sensor_qos()
        self.sub = self.create_subscription(Imu, '/imu/data', self.cb, qos)
        self.samples_target = int(samples)
        self.timeout = float(timeout)

        self.times = []
        self.angx = []
        self.angy = []
        self.angz = []
        self.accx = []
        self.accy = []
        self.accz = []

        self.start_time = time.time()
        self.get_logger().info(f'Listening on /imu/data, collecting up to {self.samples_target} samples (timeout {self.timeout}s)...')

    def cb(self, msg: Imu):
        # header stamps may be default zero — try to handle gracefully
        sec = int(msg.header.stamp.sec) if hasattr(msg.header.stamp, 'sec') else 0
        nsec = int(msg.header.stamp.nanosec) if hasattr(msg.header.stamp, 'nanosec') else 0
        t = float(sec) + float(nsec) * 1e-9
        # if stamp is zero, use wall clock (but such samples are less useful for dt)
        if t <= 0.0:
            t = time.time()
        self.times.append(t)
        self.angx.append(float(msg.angular_velocity.x))
        self.angy.append(float(msg.angular_velocity.y))
        self.angz.append(float(msg.angular_velocity.z))
        self.accx.append(float(msg.linear_acceleration.x))
        self.accy.append(float(msg.linear_acceleration.y))
        self.accz.append(float(msg.linear_acceleration.z))

        # stop conditions
        if len(self.times) >= self.samples_target:
            self.finish_and_exit()
            return
        if time.time() - self.start_time > self.timeout and len(self.times) >= 5:
            self.finish_and_exit()
            return

    def finish_and_exit(self):
        # guard
        if self.sub is None:
            return
        # unsubscribe
        try:
            self.destroy_subscription(self.sub)
        except Exception:
            pass
        self.sub = None

        n = len(self.times)
        if n < 3:
            self.get_logger().error('Too few IMU samples collected.')
            rclpy.shutdown()
            return

        # dt stats from message stamps
        dts = [self.times[i+1] - self.times[i] for i in range(n-1)]
        median_dt = statistics.median(dts)
        mean_dt = sum(dts) / len(dts)

        # per-axis medians
        ang_meds = [statistics.median([abs(x) for x in self.angx]),
                    statistics.median([abs(y) for y in self.angy]),
                    statistics.median([abs(z) for z in self.angz])]
        ang_median = statistics.median(ang_meds)
        ang_max = max(max(map(abs,self.angx)), max(map(abs,self.angy)), max(map(abs,self.angz)))

        acc_meds = [statistics.median([abs(x) for x in self.accx]),
                    statistics.median([abs(y) for y in self.accy]),
                    statistics.median([abs(z) for z in self.accz])]
        acc_median = statistics.median(acc_meds)
        norms = [math.sqrt(self.accx[i]**2 + self.accy[i]**2 + self.accz[i]**2) for i in range(n)]
        norm_median = statistics.median(norms)
        acc_max = max(max(map(abs,self.accx)), max(map(abs,self.accy)), max(map(abs,self.accz)))

        # heuristics
        ang_per_dt = ang_median / median_dt if median_dt > 0 else float('inf')
        acc_per_dt = acc_median / median_dt if median_dt > 0 else float('inf')

        guess_ang_increment = False
        if ang_median < 0.1 and 0.01 <= ang_per_dt <= 100.0:
            guess_ang_increment = True
        elif 0.01 <= ang_median <= 100.0 and not (0.01 <= ang_per_dt <= 100.0):
            guess_ang_increment = False
        else:
            if ang_max < 0.02 and (ang_max / median_dt) > 0.1:
                guess_ang_increment = True

        guess_acc_increments = False
        guess_acc_in_g = False
        if 5.0 <= norm_median <= 15.0:
            guess_acc_in_g = False
            guess_acc_increments = False
        else:
            if acc_median < 0.5 and 0.1 <= (acc_median/median_dt) <= 50.0:
                guess_acc_increments = True
            if 0.4 <= norm_median <= 1.6:
                guess_acc_in_g = True

        guess_is_increment = guess_ang_increment or guess_acc_increments

        # Prepare output
        out_lines = []
        out_lines.append(f"Collected {n} samples. median dt = {median_dt:.6f} s (mean {mean_dt:.6f} s)")
        out_lines.append(f"Angular med per-axis abs = {ang_median:.6g} rad (median), max abs = {ang_max:.6g}")
        out_lines.append(f"Angular per-sample -> ang/dt median = {ang_per_dt:.6g} rad/s")
        out_lines.append(f"Acceleration med per-axis abs = {acc_median:.6g}, median norm = {norm_median:.6g}, max abs = {acc_max:.6g}")
        out_lines.append(f"Acceleration per-sample -> acc/dt median = {acc_per_dt:.6g} m/s^2")

        rec = []
        if guess_is_increment:
            rec.append("Recommendation: imu_is_increment: true  (topic likely publishes per-sample increments / delta values)")
        else:
            rec.append("Recommendation: imu_is_increment: false (topic likely publishes rates/accelerations in SI units)")

        if guess_acc_in_g:
            rec.append("Recommendation: imu_units: 'deg_g' or 'g' (accelerometer magnitude ~1 suggests units are g).")
        else:
            rec.append("Recommendation: imu_units: 'si' (accelerations likely m/s^2 or delta-v per sample)")

        self.get_logger().info("\n".join(out_lines))
        self.get_logger().info("\n".join(rec))

        guidance = [
            "Guidance:",
            " * imu_is_increment=true means IMU msg fields are delta-angle and delta-velocity per sample (no dt multiply).",
            " * imu_is_increment=false means IMU msg fields are angular rate (rad/s) and acceleration (m/s^2) and will be multiplied by dt.",
            " * If accelerometer looks like ~1 in norm, it may be in 'g' units; set imu_units accordingly.",
            " * If conclusions contradict known sensor format, inspect the publisher/playback to see whether it publishes increments or rates."
        ]
        self.get_logger().info("\n".join(guidance))

        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', type=int, default=300, help='Number of IMU samples to collect (default 300)')
    parser.add_argument('--timeout', type=float, default=8.0, help='Timeout in seconds (default 8.0)')
    args = parser.parse_args()

    rclpy.init()
    checker = ImuChecker(samples=args.samples, timeout=args.timeout)
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        checker.get_logger().info('Interrupted by user')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
