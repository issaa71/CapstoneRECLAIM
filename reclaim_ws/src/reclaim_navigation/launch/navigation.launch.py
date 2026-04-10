"""
Full navigation launch: SLAM Toolbox + Nav2 stack.

Starts:
  - RPLIDAR A1M8 driver (/scan)
  - SLAM Toolbox in online async mode (simultaneous mapping + localization)
  - Nav2 stack (planner, controller, recoveries, BT navigator)
  - Static transform: base_link -> laser_link

This is the "do everything" launch — SLAM provides the map AND the
map->odom transform, while Nav2 plans and controls. Use this when you
don't have a pre-built map, or want to keep mapping while navigating.

Requires:
  - /odom published by the Control Agent (micro-ROS wheel odometry)

Usage:
  ros2 launch reclaim_navigation navigation.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('reclaim_navigation')

    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLIDAR A1M8'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )

    serial_port = LaunchConfiguration('serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Include RPLIDAR launch
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rplidar.launch.py')
        ),
        launch_arguments={
            'serial_port': serial_port,
        }.items(),
    )

    # SLAM Toolbox — online async (mapping + localization simultaneously)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # Nav2 controller server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/cmd_vel', '/cmd_vel')],
    )

    # Nav2 planner server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Nav2 recoveries server
    recoveries_server_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Nav2 BT navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Lifecycle manager for Nav2 nodes
    lifecycle_manager_navigation = Node(
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
    )

    return LaunchDescription([
        serial_port_arg,
        use_sim_time_arg,
        autostart_arg,
        rplidar_launch,
        slam_toolbox_node,
        controller_server_node,
        planner_server_node,
        recoveries_server_node,
        bt_navigator_node,
        lifecycle_manager_navigation,
    ])
