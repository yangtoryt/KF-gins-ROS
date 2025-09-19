cmake_minimum_required(VERSION 3.5)
project(kf_gins_node)

# 默认 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}  # 包含 src, kf-gins, common
)

# 生成可执行文件，名字叫 kf_gins_ros2_node
add_executable(kf_gins_ros2_node
  src/kf_gins_node.cpp
  kf-gins/gi_engine.cpp
  kf-gins/insmech.cpp
)

ament_target_dependencies(kf_gins_ros2_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装可执行文件
install(TARGETS
  kf_gins_ros2_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 Python 脚本
install(PROGRAMS
  scripts/playback_dataset.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
