import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('aegis_ros')
    ekf_config = os.path.join(share_dir, 'config', 'ekf.yaml')
    logger_config = os.path.join(share_dir, 'config', 'trajectory_logger.yaml')

    return LaunchDescription([
        Node(
            package='aegis_ros',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[ekf_config],
        ),
        Node(
            package='aegis_ros',
            executable='trajectory_logger_node',
            name='trajectory_logger',
            output='screen',
            parameters=[logger_config],
        ),
    ])
