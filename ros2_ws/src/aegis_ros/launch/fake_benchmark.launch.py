import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('aegis_ros')
    fake_sensor_config = os.path.join(share_dir, 'config', 'fake_sensor_publisher.yaml')
    ekf_config = os.path.join(share_dir, 'config', 'ekf.yaml')
    logger_config = os.path.join(share_dir, 'config', 'trajectory_logger.yaml')

    ld = LaunchDescription()

    fake = Node(
        package='aegis_ros',
        executable='fake_sensor_publisher_node',
        name='fake_sensor_publisher',
        output='screen',
        parameters=[
            fake_sensor_config,
            {
                'radius': 1.0,
                'omega': 0.5,
                'odom_position_noise_std': 0.03,
                'odom_velocity_noise_std': 0.02,
                'imu_yaw_rate_noise_std': 0.01,
                'dropout_probability': 0.0,
            },
        ]
    )

    ekf = Node(
        package='aegis_ros',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config],
    )

    logger = Node(
        package='aegis_ros',
        executable='trajectory_logger_node',
        name='trajectory_logger_node',
        output='screen',
        parameters=[logger_config]
    )

    ld.add_action(fake)
    ld.add_action(ekf)
    ld.add_action(logger)

    return ld
