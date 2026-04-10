"""
waste_tracker.launch.py — Launch the autonomous waste collection state machine.

Prerequisites (must be launched separately BEFORE this):
  1. teensy_bridge  — owns /dev/ttyACM0, creates /tmp/teensy_arm PTY
     ros2 launch reclaim_control teensy_bridge.launch.py

  2. Arm must be ARMed via serial command before autonomous pickup works.

This launch file starts:
  - waste_tracker node (state machine + camera + YOLO)

Usage:
  ros2 launch reclaim_bringup waste_tracker.launch.py
  ros2 launch reclaim_bringup waste_tracker.launch.py model_path:=yolov8n.pt confidence:=0.25
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('reclaim_bringup')
    config_file = os.path.join(pkg_dir, 'config', 'waste_tracker.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to waste_tracker YAML config'),

        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Override YOLO model path (empty = use config file value)'),

        DeclareLaunchArgument(
            'confidence',
            default_value='',
            description='Override confidence threshold (empty = use config file value)'),

        Node(
            package='reclaim_bringup',
            executable='waste_tracker',
            name='waste_tracker',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
