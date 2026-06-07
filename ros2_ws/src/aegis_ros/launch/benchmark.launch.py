from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aegis_ros',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=['config/ekf.yaml'],
        ),
        Node(
            package='aegis_ros',
            executable='trajectory_logger_node',
            name='trajectory_logger',
            output='screen',
        ),
    ])
