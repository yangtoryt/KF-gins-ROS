#!/usr/bin/env python3
"""
Playback script for KF-GINS dataset (IMU + GNSS).
Matches formats:

IMU line format:
  col1: time (GNSS week seconds) [s]
  col2..4: angle increments (dtheta) x,y,z [rad]
  col5..7: velocity increments (dvel) x,y,z [m/s]

GNSS line format:
  col1: time (GNSS week seconds) [s]
  col2: lat [deg]
  col3: lon [deg]
  col4: alt [m]
  col5..7: stds (north, east, down) [m]  (optional)

Usage:
  python3 playback_dataset.py \
    --imufile /path/to/Leador-A15.txt \
    --gnssfile /path/to/GNSS-RTK.txt \
    --time-scale 1.0

You may run it with your ROS2 sourced environment.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from builtin_interfaces.msg import Time as TimeMsg
import argparse
import time
import threading
import math
import sys
from typing import List, Tuple, Optional

# ---------- parsing helpers ----------
def parse_imu_line(line: str) -> Optional[Tuple[float, float, float, float, float, float, float]]:
    s = line.strip()
    if not s:
        return None
    parts = s.split()
    # tolerate header lines, comments
    if parts[0].startswith('#') or len(parts) < 7:
        return None
    try:
        t = float(parts[0])
        dtheta_x = float(parts[1])
        dtheta_y = float(parts[2])
        dtheta_z = float(parts[3])
        dvel_x = float(parts[4])
        dvel_y = float(parts[5])
        dvel_z = float(parts[6])
        return (t, dtheta_x, dtheta_y, dtheta_z, dvel_x, dvel_y, dvel_z)
    except Exception:
        return None

def parse_gnss_line(line: str) -> Optional[Tuple[float, float, float, float]]:
    s = line.strip()
    if not s:
        return None
    parts = s.split()
    if parts[0].startswith('#') or len(parts) < 4:
        return None
    try:
        t = float(parts[0])
        lat = float(parts[1])
        lon = float(parts[2])
        alt = float(parts[3])
        return (t, lat, lon, alt)
    except Exception:
        return None

def sec_to_time_msg(ds: float) -> TimeMsg:
    sec = int(math.floor(ds))
    nsec = int(round((ds - sec) * 1e9))
    # handle rounding edge-case
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    t = TimeMsg()
    t.sec = sec
    t.nanosec = nsec
    return t

# ---------- player node ----------
class DatasetPlayer(Node):
    def __init__(self, imu_file: str, gnss_file: str, time_scale: float = 1.0, loop: bool = False):
        super().__init__('kf_gins_dataset_player')
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.imu_file = imu_file
        self.gnss_file = gnss_file
        self.time_scale = float(time_scale) if time_scale > 0 else 1.0
        self.loop = loop
        self._stop = False

        self.get_logger().info(f'Loading IMU file: {imu_file}')
        self.imu_records = self._load_imu(imu_file)
        self.get_logger().info(f'Loaded {len(self.imu_records)} IMU records')

        self.get_logger().info(f'Loading GNSS file: {gnss_file}')
        self.gnss_records = self._load_gnss(gnss_file)
        self.get_logger().info(f'Loaded {len(self.gnss_records)} GNSS records')

        if len(self.imu_records) == 0 and len(self.gnss_records) == 0:
            self.get_logger().warning('No data found in both files.')

        # start the playback thread
        self.thread = threading.Thread(target=self._playback_loop, daemon=True)
        self.thread.start()

    def _load_imu(self, path: str):
        recs = []
        try:
            with open(path, 'r') as f:
                for ln in f:
                    p = parse_imu_line(ln)
                    if p:
                        recs.append(p)
        except Exception as e:
            self.get_logger().error(f'Failed to open IMU file: {e}')
        return recs

    def _load_gnss(self, path: str):
        recs = []
        try:
            with open(path, 'r') as f:
                for ln in f:
                    p = parse_gnss_line(ln)
                    if p:
                        recs.append(p)
        except Exception as e:
            self.get_logger().error(f'Failed to open GNSS file: {e}')
        return recs

    def _publish_imu(self, rec):
        t, dx, dy, dz, vx, vy, vz = rec
        msg = Imu()
        # we store increments into msg.angular_velocity (dtheta) and linear_acceleration (dvel)
        msg.header.stamp = sec_to_time_msg(t)
        msg.header.frame_id = 'imu_link'
        msg.angular_velocity.x = dx
        msg.angular_velocity.y = dy
        msg.angular_velocity.z = dz
        msg.linear_acceleration.x = vx
        msg.linear_acceleration.y = vy
        msg.linear_acceleration.z = vz
        self.imu_pub.publish(msg)

    def _publish_gnss(self, rec):
        t, lat, lon, alt = rec
        msg = NavSatFix()
        msg.header.stamp = sec_to_time_msg(t)
        msg.header.frame_id = 'map'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        # leave covariance at defaults (0) — your node may ignore them or compute from std columns
        self.gnss_pub.publish(msg)

    def _playback_loop(self):
        # if both empty -> nothing to do
        if len(self.imu_records) == 0 and len(self.gnss_records) == 0:
            return

        while rclpy.ok() and not self._stop:
            imu_idx = 0
            gnss_idx = 0
            if len(self.imu_records) > 0:
                start_t = self.imu_records[0][0]
            elif len(self.gnss_records) > 0:
                start_t = self.gnss_records[0][0]
            else:
                break

            wall_start = time.time()
            last_print = wall_start

            # iterate until both exhausted
            while (imu_idx < len(self.imu_records) or gnss_idx < len(self.gnss_records)) and rclpy.ok() and not self._stop:
                now_wall = time.time()
                elapsed_wall = now_wall - wall_start
                # map to dataset time
                target_ds_time = start_t + elapsed_wall * self.time_scale

                # publish IMU records up to target_ds_time
                while imu_idx < len(self.imu_records) and self.imu_records[imu_idx][0] <= target_ds_time:
                    self._publish_imu(self.imu_records[imu_idx])
                    imu_idx += 1

                # publish GNSS records up to target_ds_time
                while gnss_idx < len(self.gnss_records) and self.gnss_records[gnss_idx][0] <= target_ds_time:
                    self._publish_gnss(self.gnss_records[gnss_idx])
                    gnss_idx += 1

                # progress log occasionally
                if now_wall - last_print > 1.0:
                    perc_imu = (imu_idx / len(self.imu_records) * 100) if self.imu_records else 100
                    perc_gnss = (gnss_idx / len(self.gnss_records) * 100) if self.gnss_records else 100
                    self.get_logger().info(f'playback: imu {imu_idx}/{len(self.imu_records)} ({perc_imu:.1f}%), gnss {gnss_idx}/{len(self.gnss_records)} ({perc_gnss:.1f}%)')
                    last_print = now_wall

                # tiny sleep
                time.sleep(0.0005)

            self.get_logger().info('Playback pass finished.')
            if not self.loop:
                break
            self.get_logger().info('Looping playback: restarting.')

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser(description='KF-GINS dataset playback (IMU+GNSS)')
    parser.add_argument('--imufile', type=str, required=False,
                        default='/home/yang/KF-ROS/ros_ws/src/kf_gins_node/config/dataset/Leador-A15.txt')
    parser.add_argument('--gnssfile', type=str, required=False,
                        default='/home/yang/KF-ROS/ros_ws/src/kf_gins_node/config/dataset/GNSS-RTK.txt')
    parser.add_argument('--time-scale', type=float, default=1.0,
                        help='Playback speed: 1.0 = real-time, >1 faster, <1 slower')
    parser.add_argument('--loop', action='store_true', help='Loop playback')
    args = parser.parse_args(argv)

    rclpy.init()
    player = DatasetPlayer(args.imufile, args.gnssfile, time_scale=args.time_scale, loop=args.loop)
    try:
        rclpy.spin(player)
    except KeyboardInterrupt:
        pass
    player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
