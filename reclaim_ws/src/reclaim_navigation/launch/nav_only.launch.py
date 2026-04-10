"""
Launch file for Nav2 navigation with a pre-built map.

Starts:
  - RPLIDAR A1M8 driver (/scan)
  - Map server (loads saved .yaml/.pgm map)
  - AMCL (localization against the static map)
  - Nav2 stack (planner, controller, recoveries, BT navigator)
  - Static transform: base_link -> laser_link

Requires:
  - A pre-built map file (use slam_only.launch.py + map_saver_cli first)
  - /odom published by the Control Agent (micro-ROS wheel odometry)

Usage:
  ros2 launch reclaim_navigation nav_only.launch.py map:=/path/to/venue_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('reclaim_navigation')

    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_dir, 'maps', 'venue_map.yaml')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to map .yaml file'
    )

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

    map_file = LaunchConfiguration('map')
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

    # Map server — serves the pre-built occupancy grid
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time},
        ],
    )

    # AMCL — particle filter localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Lifecycle manager for localization (map_server + amcl)
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
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

    # Lifecycle manager for Nav2 (controller, planner, recoveries, bt_navigator)
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
        map_arg,
        serial_port_arg,
        use_sim_time_arg,
        autostart_arg,
        rplidar_launch,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        controller_server_node,
        planner_server_node,
        recoveries_server_node,
        bt_navigator_node,
        lifecycle_manager_navigation,
    ])
