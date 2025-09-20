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

## ** 1. 编译 KF-GINS 核心库 **
1. 克隆核心库源码：
   ```bash
   git clone https://github.com/i2NavGroup/KF-GINS.git
   cd KF-GINS
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/install
   make -j4
   make install
   export KF_GINS_ROOT=/path/to/install  # 与 ROS 包编译关联
   ```

#### ** 2. 添加参数说明表 **
## 核心参数说明
| 参数名              | 单位       | 说明                                  | 推荐值       |
|---------------------|------------|---------------------------------------|--------------|
| imu_is_increment    | -          | IMU 数据是否为增量（delta）           | false（速率）|
| imu_units           | -          | 单位类型（si/deg_g）                  | si           |
| imunoise.arw        | rad/√s     | 角速度随机游走                        | 0.003        |
| initpos             | deg,deg,m  | 初始位置（纬度、经度、高度）          | 测试区域坐标 |

## 测试流程
1. 准备数据集：将 IMU 和 GPS 数据放入 `kf_gins_node/config/dataset/`
2. 检查 IMU 格式（运行工具）：
   ```bash
   ros2 run kf_gins_node imu_check.py
   ros2 launch kf_gins_ros2 kf_gins_launch.py
   rviz2 -d kf_gins.rviz


### ** 4.许可证与代码规范（影响合规性）**
#### 1. **统一许可证**
核心库 `insmech.cpp` 为 GPLv3，而 `package.xml` 为 MIT，存在冲突。需修改 `package.xml`：
```xml
<license>GPLv3</license>  <!-- 与核心库保持一致 -->
