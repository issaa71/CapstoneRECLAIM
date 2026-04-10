"""
Launch file for SLAM mapping mode.

Starts:
  - RPLIDAR A1M8 driver (/scan)
  - SLAM Toolbox in online async mapping mode
  - Static transform: base_link -> laser_link

Use this to drive the robot around manually and build a map.
Save the map afterward with:
  ros2 run nav2_map_server map_saver_cli -f ~/reclaim_ws/src/reclaim_navigation/maps/venue_map

Usage:
  ros2 launch reclaim_navigation slam_only.launch.py
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

    serial_port = LaunchConfiguration('serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include RPLIDAR launch (starts driver + base_link->laser_link TF)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rplidar.launch.py')
        ),
        launch_arguments={
            'serial_port': serial_port,
        }.items(),
    )

    # SLAM Toolbox — online async mapping
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

    return LaunchDescription([
        serial_port_arg,
        use_sim_time_arg,
        rplidar_launch,
        slam_toolbox_node,
    ])
