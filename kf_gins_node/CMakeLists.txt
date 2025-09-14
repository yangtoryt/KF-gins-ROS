cmake_minimum_required(VERSION 3.5)
project(kf_gins_node)

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

add_executable(kf_gins_node
  src/kf_gins_node.cpp
  kf-gins/gi_engine.cpp
  kf-gins/insmech.cpp
)

ament_target_dependencies(kf_gins_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS kf_gins_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
