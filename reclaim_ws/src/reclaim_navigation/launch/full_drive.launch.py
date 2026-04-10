"""
Full driving + navigation launch for RECLAIM robot.

Starts (in order):
  1. Teensy bridge — serial owner, /cmd_vel subscriber, /odom publisher, PTY proxy
  2. RPLIDAR A1M8 driver — /scan publisher (delayed 3s to ensure PTY exists)
  3. SLAM Toolbox — online async mapping + localization
  4. Nav2 stack — path planning, local control, recoveries

MoveIt2 arm control is launched separately. It connects to
/tmp/teensy_arm (PTY created by teensy_bridge).

Requires:
  - Teensy firmware with DRIVE/TICKS commands
  - rplidar_ros installed (conda install -c robostack-staging ros-humble-rplidar-ros)

Usage:
  ros2 launch reclaim_navigation full_drive.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_pkg_dir = get_package_share_directory('reclaim_navigation')
    ctrl_pkg_dir = get_package_share_directory('reclaim_control')

    slam_params_file = os.path.join(nav_pkg_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml')

    # --- Launch arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
    )
    rplidar_port_arg = DeclareLaunchArgument(
        'rplidar_port', default_value='/dev/ttyUSB0',
    )
    teensy_port_arg = DeclareLaunchArgument(
        'teensy_port', default_value='/dev/ttyACM0',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # --- 1. Teensy Bridge (must start first — creates PTY) ---
    teensy_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg_dir, 'launch', 'teensy_bridge.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('teensy_port'),
        }.items(),
    )

    # --- 2. RPLIDAR (delayed 3s to ensure bridge + PTY are ready) ---
    rplidar_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg_dir, 'launch', 'rplidar.launch.py')
                ),
                launch_arguments={
                    'serial_port': LaunchConfiguration('rplidar_port'),
                }.items(),
            ),
        ],
    )

    # --- 3. SLAM Toolbox ---
    slam_toolbox_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time},
                ],
                output='screen',
            ),
        ],
    )

    # --- 4. Nav2 stack ---
    nav2_nodes = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('/cmd_vel', '/cmd_vel')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'recoveries_server',
                        'bt_navigator',
                    ],
                }],
            ),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        rplidar_port_arg,
        teensy_port_arg,
        teensy_bridge_launch,
        rplidar_launch,
        slam_toolbox_node,
        nav2_nodes,
    ])
