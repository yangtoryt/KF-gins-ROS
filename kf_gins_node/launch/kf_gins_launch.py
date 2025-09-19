import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 找到配置文件路径
    config = os.path.join(
        get_package_share_directory('kf_gins_node'),
        'config',
        'kf-gins.yaml'
    )

    return LaunchDescription([
        Node(
            package='kf_gins_node',
            executable='kf_gins_ros2_node',
            name='kf_gins_node',
            parameters=[config],
            output='screen'
        )
    ])
