import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('aegis_ros')
    fake_sensor_config = os.path.join(share_dir, 'config', 'fake_sensor_publisher.yaml')
    ekf_config = os.path.join(share_dir, 'config', 'ekf.yaml')
    ukf_config = os.path.join(share_dir, 'config', 'ukf.yaml')
    pf_config = os.path.join(share_dir, 'config', 'particle_filter.yaml')
    logger_config = os.path.join(share_dir, 'config', 'trajectory_logger.yaml')

    run_ekf = LaunchConfiguration('run_ekf')
    run_ukf = LaunchConfiguration('run_ukf')
    run_pf = LaunchConfiguration('run_pf')
    odom_position_noise_std = LaunchConfiguration('odom_position_noise_std')
    odom_velocity_noise_std = LaunchConfiguration('odom_velocity_noise_std')
    imu_yaw_rate_noise_std = LaunchConfiguration('imu_yaw_rate_noise_std')
    dropout_probability = LaunchConfiguration('dropout_probability')
    benchmark_duration_seconds = LaunchConfiguration('benchmark_duration_seconds')
    fake_sensor_seed = LaunchConfiguration('fake_sensor_seed')
    pf_random_seed = LaunchConfiguration('pf_random_seed')
    use_odom_pose_update = LaunchConfiguration('use_odom_pose_update')
    pose_gating_enabled = LaunchConfiguration('pose_gating_enabled')
    pose_gating_threshold = LaunchConfiguration('pose_gating_threshold')
    pose_outlier_probability = LaunchConfiguration('pose_outlier_probability')
    pose_outlier_position_std = LaunchConfiguration('pose_outlier_position_std')
    pose_outlier_yaw_std = LaunchConfiguration('pose_outlier_yaw_std')
    pose_outlier_start_seconds = LaunchConfiguration('pose_outlier_start_seconds')
    correction_enabled = LaunchConfiguration('correction_enabled')
    correction_replay_enabled = LaunchConfiguration('correction_replay_enabled')
    correction_start_seconds = LaunchConfiguration('correction_start_seconds')
    correction_frequency_hz = LaunchConfiguration('correction_frequency_hz')
    correction_dropout_probability = LaunchConfiguration('correction_dropout_probability')
    correction_blackout_start_seconds = LaunchConfiguration('correction_blackout_start_seconds')
    correction_blackout_duration_seconds = LaunchConfiguration('correction_blackout_duration_seconds')
    correction_latency_seconds = LaunchConfiguration('correction_latency_seconds')
    correction_latency_schedule_seconds = LaunchConfiguration('correction_latency_schedule_seconds')
    correction_max_emissions = LaunchConfiguration('correction_max_emissions')
    correction_position_noise_std = LaunchConfiguration('correction_position_noise_std')
    correction_yaw_noise_std = LaunchConfiguration('correction_yaw_noise_std')
    correction_outlier_probability = LaunchConfiguration('correction_outlier_probability')
    correction_outlier_position_std = LaunchConfiguration('correction_outlier_position_std')
    correction_outlier_yaw_std = LaunchConfiguration('correction_outlier_yaw_std')
    results_dir = LaunchConfiguration('results_dir')
    fake_sensor_stats_out = LaunchConfiguration('fake_sensor_stats_out')
    corruption_log_out = LaunchConfiguration('corruption_log_out')
    correction_log_out = LaunchConfiguration('correction_log_out')
    ekf_stats_out = LaunchConfiguration('ekf_stats_out')
    ukf_stats_out = LaunchConfiguration('ukf_stats_out')
    pf_stats_out = LaunchConfiguration('pf_stats_out')
    logger_stats_out = LaunchConfiguration('logger_stats_out')
    max_history_seconds = LaunchConfiguration('max_history_seconds')

    ld = LaunchDescription([
        DeclareLaunchArgument('run_ekf', default_value='true'),
        DeclareLaunchArgument('run_ukf', default_value='true'),
        DeclareLaunchArgument('run_pf', default_value='true'),
        DeclareLaunchArgument('odom_position_noise_std', default_value='0.03'),
        DeclareLaunchArgument('odom_velocity_noise_std', default_value='0.02'),
        DeclareLaunchArgument('imu_yaw_rate_noise_std', default_value='0.01'),
        DeclareLaunchArgument('dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('benchmark_duration_seconds', default_value='30'),
        DeclareLaunchArgument('fake_sensor_seed', default_value='1337'),
        DeclareLaunchArgument('pf_random_seed', default_value='4242'),
        DeclareLaunchArgument('use_odom_pose_update', default_value='true'),
        DeclareLaunchArgument('pose_gating_enabled', default_value='false'),
        DeclareLaunchArgument('pose_gating_threshold', default_value='9.324146034653893'),
        DeclareLaunchArgument('pose_outlier_probability', default_value='0.0'),
        DeclareLaunchArgument('pose_outlier_position_std', default_value='1.5'),
        DeclareLaunchArgument('pose_outlier_yaw_std', default_value='0.75'),
        DeclareLaunchArgument('pose_outlier_start_seconds', default_value='0.0'),
        DeclareLaunchArgument('correction_enabled', default_value='false'),
        DeclareLaunchArgument('correction_replay_enabled', default_value='true'),
        DeclareLaunchArgument('correction_start_seconds', default_value='0.0'),
        DeclareLaunchArgument('correction_frequency_hz', default_value='2.0'),
        DeclareLaunchArgument('correction_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('correction_blackout_start_seconds', default_value='-1.0'),
        DeclareLaunchArgument('correction_blackout_duration_seconds', default_value='0.0'),
        DeclareLaunchArgument('correction_latency_seconds', default_value='0.0'),
        DeclareLaunchArgument('correction_latency_schedule_seconds', default_value=''),
        DeclareLaunchArgument('correction_max_emissions', default_value='0'),
        DeclareLaunchArgument('correction_position_noise_std', default_value='0.05'),
        DeclareLaunchArgument('correction_yaw_noise_std', default_value='0.08'),
        DeclareLaunchArgument('correction_outlier_probability', default_value='0.0'),
        DeclareLaunchArgument('correction_outlier_position_std', default_value='1.5'),
        DeclareLaunchArgument('correction_outlier_yaw_std', default_value='0.75'),
        DeclareLaunchArgument('results_dir', default_value='results/metrics'),
        DeclareLaunchArgument('fake_sensor_stats_out', default_value=''),
        DeclareLaunchArgument('corruption_log_out', default_value=''),
        DeclareLaunchArgument('correction_log_out', default_value=''),
        DeclareLaunchArgument('ekf_stats_out', default_value=''),
        DeclareLaunchArgument('ukf_stats_out', default_value=''),
        DeclareLaunchArgument('pf_stats_out', default_value=''),
        DeclareLaunchArgument('logger_stats_out', default_value=''),
        DeclareLaunchArgument('max_history_seconds', default_value='5.0'),
    ])

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
                'duration_seconds': benchmark_duration_seconds,
                'random_seed': fake_sensor_seed,
                'odom_position_noise_std': odom_position_noise_std,
                'odom_velocity_noise_std': odom_velocity_noise_std,
                'imu_yaw_rate_noise_std': imu_yaw_rate_noise_std,
                'dropout_probability': dropout_probability,
                'pose_outlier_probability': pose_outlier_probability,
                'pose_outlier_position_std': pose_outlier_position_std,
                'pose_outlier_yaw_std': pose_outlier_yaw_std,
                'pose_outlier_start_seconds': pose_outlier_start_seconds,
                'correction_enabled': correction_enabled,
                'correction_start_seconds': correction_start_seconds,
                'correction_frequency_hz': correction_frequency_hz,
                'correction_dropout_probability': correction_dropout_probability,
                'correction_blackout_start_seconds': correction_blackout_start_seconds,
                'correction_blackout_duration_seconds': correction_blackout_duration_seconds,
                'correction_latency_seconds': correction_latency_seconds,
                'correction_latency_schedule_seconds': correction_latency_schedule_seconds,
                'correction_max_emissions': correction_max_emissions,
                'correction_position_noise_std': correction_position_noise_std,
                'correction_yaw_noise_std': correction_yaw_noise_std,
                'correction_outlier_probability': correction_outlier_probability,
                'correction_outlier_position_std': correction_outlier_position_std,
                'correction_outlier_yaw_std': correction_outlier_yaw_std,
                'stats_out': fake_sensor_stats_out,
                'corruption_log_out': corruption_log_out,
                'correction_log_out': correction_log_out,
            },
        ]
    )

    ekf = Node(
        package='aegis_ros',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config, {
            'use_odom_pose_update': use_odom_pose_update,
            'pose_gating_enabled': pose_gating_enabled,
            'pose_gating_threshold': pose_gating_threshold,
            'max_history_seconds': max_history_seconds,
            'correction_replay_enabled': correction_replay_enabled,
            'stats_out': ekf_stats_out,
        }],
        condition=IfCondition(run_ekf),
    )

    ukf = Node(
        package='aegis_ros',
        executable='ukf_node',
        name='ukf_node',
        output='screen',
        parameters=[ukf_config, {
            'use_odom_pose_update': use_odom_pose_update,
            'pose_gating_enabled': pose_gating_enabled,
            'pose_gating_threshold': pose_gating_threshold,
            'max_history_seconds': max_history_seconds,
            'correction_replay_enabled': correction_replay_enabled,
            'stats_out': ukf_stats_out,
        }],
        condition=IfCondition(run_ukf),
    )

    pf = Node(
        package='aegis_ros',
        executable='particle_filter_node',
        name='particle_filter_node',
        output='screen',
        parameters=[pf_config, {
            'use_odom_pose_update': use_odom_pose_update,
            'max_history_seconds': max_history_seconds,
            'correction_replay_enabled': correction_replay_enabled,
            'random_seed': pf_random_seed,
            'stats_out': pf_stats_out,
        }],
        condition=IfCondition(run_pf),
    )

    logger = Node(
        package='aegis_ros',
        executable='trajectory_logger_node',
        name='trajectory_logger_node',
        output='screen',
        parameters=[logger_config, {'results_dir': results_dir, 'stats_out': logger_stats_out}]
    )

    ld.add_action(fake)
    ld.add_action(ekf)
    ld.add_action(ukf)
    ld.add_action(pf)
    ld.add_action(logger)

    return ld
