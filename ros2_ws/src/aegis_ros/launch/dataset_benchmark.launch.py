import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('aegis_ros')
    ekf_config = os.path.join(share_dir, 'config', 'ekf.yaml')
    ukf_config = os.path.join(share_dir, 'config', 'ukf.yaml')
    pf_config = os.path.join(share_dir, 'config', 'particle_filter.yaml')
    logger_config = os.path.join(share_dir, 'config', 'trajectory_logger.yaml')

    run_ekf = LaunchConfiguration('run_ekf')
    run_ukf = LaunchConfiguration('run_ukf')
    run_pf = LaunchConfiguration('run_pf')
    use_odom_pose_update = LaunchConfiguration('use_odom_pose_update')
    results_dir = LaunchConfiguration('results_dir')
    log_odom_baseline = LaunchConfiguration('log_odom_baseline')
    ekf_stats_out = LaunchConfiguration('ekf_stats_out')
    ukf_stats_out = LaunchConfiguration('ukf_stats_out')
    pf_stats_out = LaunchConfiguration('pf_stats_out')
    logger_stats_out = LaunchConfiguration('logger_stats_out')

    return LaunchDescription([
        DeclareLaunchArgument('run_ekf', default_value='true'),
        DeclareLaunchArgument('run_ukf', default_value='true'),
        DeclareLaunchArgument('run_pf', default_value='true'),
        DeclareLaunchArgument('use_odom_pose_update', default_value='false'),
        DeclareLaunchArgument('results_dir', default_value='results/metrics'),
        DeclareLaunchArgument('log_odom_baseline', default_value='false'),
        DeclareLaunchArgument('ekf_stats_out', default_value=''),
        DeclareLaunchArgument('ukf_stats_out', default_value=''),
        DeclareLaunchArgument('pf_stats_out', default_value=''),
        DeclareLaunchArgument('logger_stats_out', default_value=''),
        Node(
            package='aegis_ros',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config, {'use_odom_pose_update': use_odom_pose_update, 'stats_out': ekf_stats_out}],
            condition=IfCondition(run_ekf),
        ),
        Node(
            package='aegis_ros',
            executable='ukf_node',
            name='ukf_node',
            output='screen',
            parameters=[ukf_config, {'use_odom_pose_update': use_odom_pose_update, 'stats_out': ukf_stats_out}],
            condition=IfCondition(run_ukf),
        ),
        Node(
            package='aegis_ros',
            executable='particle_filter_node',
            name='particle_filter_node',
            output='screen',
            parameters=[pf_config, {'use_odom_pose_update': use_odom_pose_update, 'stats_out': pf_stats_out}],
            condition=IfCondition(run_pf),
        ),
        Node(
            package='aegis_ros',
            executable='trajectory_logger_node',
            name='trajectory_logger_node',
            output='screen',
            parameters=[logger_config, {'results_dir': results_dir, 'log_odom_baseline': log_odom_baseline, 'stats_out': logger_stats_out}],
        ),
    ])
