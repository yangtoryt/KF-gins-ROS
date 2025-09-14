# kf_gins_ros2

ROS2 Foxy wrapper for KF-GINS core library.

## Overview

This package expects a library `libkf_gins_core` provided by the original KF-GINS project.
The node calls C-style wrapper functions (kf_gins_create_from_config, kf_gins_feed_imu, etc.)
which you should implement in KF-GINS core (or provide a small api_wrapper).

## Build & Run

1. Build and install KF-GINS core library (libkf_gins_core) into /usr/local or set KF_GINS_ROOT env var.
2. Build this package:

   ```bash
   source /opt/ros/foxy/setup.bash
   cd ~/KF-ROS/ros_ws
   colcon build --symlink-install
   source ~/KF-ROS/ros_ws/install/setup.bash
   ros2 launch kf_gins_ros2 kf_gins_launch.py
   ```

## Notes

- If KF-GINS core is installed in a non-standard location, set:
  `export KF_GINS_ROOT=/path/to/KF-GINS/install_or_build`
