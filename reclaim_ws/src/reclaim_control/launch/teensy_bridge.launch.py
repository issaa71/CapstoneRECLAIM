"""
Launch the Teensy bridge node with parameters from teensy_bridge.yaml.

The bridge creates a PTY at /tmp/teensy_arm before spin(), so downstream
nodes (like servo_hardware_interface) can connect to it immediately.

Note: The LiDAR static TF (base_link -> laser_link) is published by
rplidar.launch.py in reclaim_navigation, not here.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('reclaim_control')
    params_file = os.path.join(pkg_dir, 'config', 'teensy_bridge.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Teensy 4.1',
    )

    teensy_bridge_node = Node(
        package='reclaim_control',
        executable='teensy_bridge',
        name='teensy_bridge',
        output='screen',
        parameters=[
            params_file,
            {'serial_port': LaunchConfiguration('serial_port')},
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        teensy_bridge_node,
    ])
