"""
Launch file to visualize the RECLAIM arm URDF in RViz2.

Usage (on MIC-711):
  cd ~/reclaim_ws
  source install/setup.bash  # if built with colcon
  ros2 launch reclaim_control view_arm.launch.py

Or without building (standalone):
  ros2 launch ~/reclaim_ws/src/reclaim_control/launch/view_arm.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Path to URDF file
    urdf_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'urdf', 'reclaim_arm.urdf'
    )

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Publish robot description to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # GUI sliders to move each joint
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'config', 'view_arm.rviz'
            )],
            output='screen'
        ),
    ])
