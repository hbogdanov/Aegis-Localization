import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('aegis_ros'),
        'config',
        'ukf.yaml'
    )

    return LaunchDescription([
        Node(
            package='aegis_ros',
            executable='ukf_node',
            name='ukf_localization',
            output='screen',
            parameters=[config_file],
        )
    ])
