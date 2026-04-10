"""
Launch file for RPLIDAR A1M8 standalone.

Starts:
  - rplidar_node: publishes /scan (sensor_msgs/LaserScan)
  - static_transform: base_link -> laser_link

Usage:
  ros2 launch reclaim_navigation rplidar.launch.py
  ros2 launch reclaim_navigation rplidar.launch.py serial_port:=/dev/ttyUSB1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('reclaim_navigation')

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLIDAR A1M8'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_link',
        description='TF frame ID for laser scans'
    )

    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')

    # RPLIDAR driver node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 115200,
            'frame_id': frame_id,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': '',
        }],
        output='screen',
    )

    # Static transform: base_link -> laser_link
    # Adjust x, y, z based on actual LiDAR mounting position on chassis
    # Assuming LiDAR is mounted on top-center of robot, ~0.15m above base_link
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.15',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '3.14159',   # LiDAR may be mounted facing backward — adjust as needed
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_link',
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        rplidar_node,
        laser_tf,
    ])
