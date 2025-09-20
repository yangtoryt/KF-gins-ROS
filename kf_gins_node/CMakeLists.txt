cmake_minimum_required(VERSION 3.5)
project(kf_gins_node)

if(NOT DEFINED KF_GINS_ROOT AND NOT EXISTS "/usr/local/include/kf_gins")
  message(FATAL_ERROR "KF_GINS_ROOT is not set and core library not found in /usr/local. \
  Please export KF_GINS_ROOT=/path/to/kf-gins/install")
endif()

# 补充核心库头文件路径
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${KF_GINS_ROOT}/include  # 添加核心库头文件路径
)

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

# 查找 kf_gins 核心库
find_library(KF_GINS_CORE 
  NAMES kf_gins_core 
  HINTS ${KF_GINS_ROOT}/lib /usr/local/lib
)
if(NOT KF_GINS_CORE)
  message(FATAL_ERROR "libkf_gins_core not found. Set KF_GINS_ROOT or install to system path.")
endif()


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

target_link_libraries(kf_gins_ros2_node 
  ${KF_GINS_CORE}#链接核心库
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


