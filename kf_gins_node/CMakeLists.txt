cmake_minimum_required(VERSION 3.5)
project(kf_gins_node)

# 先导入系统环境变量 KF_GINS_ROOT（之前的关键修复）
set(KF_GINS_ROOT $ENV{KF_GINS_ROOT})

# 检查 KF_GINS_ROOT 是否有效
if(NOT DEFINED KF_GINS_ROOT AND NOT EXISTS "/usr/local/include/kf_gins")
  message(FATAL_ERROR "KF_GINS_ROOT is not set and core library not found in /usr/local. \
  Please export KF_GINS_ROOT=/path/to/kf-gins/install")
endif()

# 查找 Eigen3（必须在 include_directories 前执行）
find_package(Eigen3 REQUIRED)
# 打印 Eigen3 路径（调试用，确认是否找到）
message("Eigen3 include directory: ${EIGEN3_INCLUDE_DIRS}")

# 统一设置包含目录（合并重复的 include_directories，确保 Eigen 路径生效）
include_directories(
  ${EIGEN3_INCLUDE_DIRS}  # Eigen3 头文件路径（关键）
  ${CMAKE_CURRENT_SOURCE_DIR}  # 项目源码根目录（包含 src、kf-gins 等）
  ${KF_GINS_ROOT}/include  # KF-GINS 核心库头文件路径
)

# 其他配置保持不变...
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)  # 新增：查找 ament_index_cpp 包

# 查找 kf_gins 核心库
find_library(KF_GINS_CORE 
  NAMES kf_gins_core 
  HINTS ${KF_GINS_ROOT}/lib /usr/local/lib
)
if(NOT KF_GINS_CORE)
  message(FATAL_ERROR "libkf_gins_core not found. Set KF_GINS_ROOT or install to system path.")
endif()

# 默认 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 生成可执行文件
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
  ament_index_cpp
)

target_link_libraries(kf_gins_ros2_node 
  ${KF_GINS_CORE}
)

# 安装配置（保持不变）
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  kf_gins_ros2_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/playback_dataset.py
  scripts/find_bad_imu.py 
  scripts/imu_check.py 
  scripts/imu_echo.py
  config/dataset/check_time_range.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()