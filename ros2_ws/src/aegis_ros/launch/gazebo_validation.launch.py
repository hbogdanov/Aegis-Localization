import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    aegis_share = get_package_share_directory('aegis_ros')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_share = get_package_share_directory('gazebo_ros')

    ekf_config = os.path.join(aegis_share, 'config', 'ekf.yaml')
    ukf_config = os.path.join(aegis_share, 'config', 'ukf.yaml')
    pf_config = os.path.join(aegis_share, 'config', 'particle_filter.yaml')
    logger_config = os.path.join(aegis_share, 'config', 'trajectory_logger.yaml')
    ground_truth_model = os.path.join(aegis_share, 'models', 'turtlebot3_burger_ground_truth.sdf')

    run_ekf = LaunchConfiguration('run_ekf')
    run_ukf = LaunchConfiguration('run_ukf')
    run_pf = LaunchConfiguration('run_pf')
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    use_odom_pose_update = LaunchConfiguration('use_odom_pose_update')

    world = os.path.join(tb3_share, 'worlds', 'empty_world.world')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_share, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model,
            '-file', ground_truth_model,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('run_ekf', default_value='true'),
        DeclareLaunchArgument('run_ukf', default_value='true'),
        DeclareLaunchArgument('run_pf', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('model', default_value='burger'),
        DeclareLaunchArgument('use_odom_pose_update', default_value='true'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', model),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_turtlebot,
        Node(
            package='aegis_ros',
            executable='gazebo_ground_truth_bridge_node',
            name='gazebo_ground_truth_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': True, 'ground_truth_odom_topic': '/ground_truth/odom'}],
        ),
        Node(
            package='aegis_ros',
            executable='circle_command_publisher_node',
            name='circle_command_publisher_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='aegis_ros',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': True, 'use_odom_pose_update': use_odom_pose_update}],
            condition=IfCondition(run_ekf),
        ),
        Node(
            package='aegis_ros',
            executable='ukf_node',
            name='ukf_node',
            output='screen',
            parameters=[ukf_config, {'use_sim_time': True, 'use_odom_pose_update': use_odom_pose_update}],
            condition=IfCondition(run_ukf),
        ),
        Node(
            package='aegis_ros',
            executable='particle_filter_node',
            name='particle_filter_node',
            output='screen',
            parameters=[pf_config, {'use_sim_time': True, 'use_odom_pose_update': use_odom_pose_update}],
            condition=IfCondition(run_pf),
        ),
        Node(
            package='aegis_ros',
            executable='trajectory_logger_node',
            name='trajectory_logger_node',
            output='screen',
            parameters=[logger_config, {'use_sim_time': True, 'results_dir': 'results/gazebo_metrics'}],
        ),
    ])
