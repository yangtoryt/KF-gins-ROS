from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kf_gins_ros2',
            executable='kf_gins_node',
            name='kf_gins_node',
            output='screen',
            parameters=[{'config': '/home/yang/KF-GINS/dataset/kf-gins.yaml'}]
        )
    ])
