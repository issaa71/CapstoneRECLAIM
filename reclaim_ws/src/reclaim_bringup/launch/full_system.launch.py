"""
full_system.launch.py — Single command to start RECLAIM demo system.

Starts everything EXCEPT waste_tracker (run that manually to start/stop the robot).

Timeline:
  t=0s   Serial permissions + Teensy bridge
  t=4s   MoveIt2 move_group + TRT server (parallel)
  t=12s  Camera bridge (needs TRT server ready)
  t=13s  Foxglove bridge
  t=20s  Load planning scene (needs move_group ready)
  t=24s  Pickup listener (needs move_group + planning scene)

Usage:
  # On MIC-711 desktop (needs DISPLAY=:0 for RViz):
  export DISPLAY=:0 && ros2 launch reclaim_bringup full_system.launch.py

  # Then in a separate terminal, start/stop the robot:
  ros2 run reclaim_bringup waste_tracker_trt --ros-args -p trt_url:=http://localhost:8081 -p drive_only:=false
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Package paths ──────────────────────────────────────────
    bringup_dir = get_package_share_directory('reclaim_bringup')
    moveit_dir = get_package_share_directory('reclaim_arm_moveit_config')

    # ── Launch arguments ───────────────────────────────────────
    trt_model_arg = DeclareLaunchArgument(
        'trt_model',
        default_value=os.path.expanduser(
            '~/reclaim_ws/models/waste_yolo26n_v6_best.engine'),
        description='Path to TRT engine file')

    trt_conf_arg = DeclareLaunchArgument(
        'trt_conf',
        default_value='0.35',
        description='TRT server confidence threshold')

    trt_port_arg = DeclareLaunchArgument(
        'trt_port',
        default_value='8081',
        description='TRT server port')

    # ── t=0s: Serial permissions + Teensy bridge ───────────────
    serial_perms = ExecuteProcess(
        cmd=['bash', '-c',
             "echo 'mic-711' | sudo -S chmod 666 /dev/ttyACM0"],
        output='screen',
        name='serial_perms',
    )

    teensy_bridge = Node(
        package='reclaim_control',
        executable='teensy_bridge',
        name='teensy_bridge',
        output='screen',
    )

    # ── t=4s: MoveIt2 move_group ───────────────────────────────
    # Needs teensy_bridge PTY at /tmp/teensy_arm — bridge takes ~2s
    move_group = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_dir, 'launch', 'move_group.launch.py')
                ),
            ),
        ],
    )

    # ── t=4s: TRT server (Python 3.8, isolated from conda) ────
    trt_server = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'kill $(lsof -t -i:8081) 2>/dev/null; '
                     'env -i HOME=/home/mic-711 '
                     'PATH=/usr/bin:/usr/local/bin:/bin '
                     'OMP_NUM_THREADS=1 '
                     'python3.8 ~/reclaim_ws/tests/camera_detect_trt.py '
                     '--model ~/reclaim_ws/models/waste_yolo26n_v6_best.engine '
                     '--conf 0.35 --port 8081'],
                output='screen',
                name='trt_server',
            ),
        ],
    )

    # ── t=12s: Camera bridge (HTTP /frame → ROS2 image topic) ──
    # Needs LD_PRELOAD for libgomp (cv2 TLS fix on aarch64)
    camera_bridge = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='reclaim_bringup',
                executable='camera_bridge',
                name='camera_bridge',
                output='screen',
                parameters=[{
                    'trt_url': 'http://localhost:8081',
                    'rate_hz': 15.0,
                }],
                additional_env={
                    'LD_PRELOAD': '/home/mic-711/miniforge3/envs/ros_env/lib/libgomp.so.1',
                },
            ),
        ],
    )

    # ── t=13s: Foxglove bridge ─────────────────────────────────
    foxglove_bridge = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[{'port': 8765}],
            ),
        ],
    )

    # ── t=20s: Load planning scene (collision objects) ──────────
    # Needs move_group ready
    planning_scene = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'source ~/miniforge3/etc/profile.d/conda.sh && '
                     'conda activate ros_env && '
                     'cd ~/reclaim_ws && source install/setup.bash && '
                     'python3 src/reclaim_arm_moveit_config/scripts/load_planning_scene.py'],
                output='screen',
                name='planning_scene',
            ),
        ],
    )

    # ── t=24s: Pickup listener (arm control) ───────────────────
    # Needs move_group + planning scene ready
    pickup_listener = TimerAction(
        period=24.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'source ~/miniforge3/etc/profile.d/conda.sh && '
                     'conda activate ros_env && '
                     'cd ~/reclaim_ws && source install/setup.bash && '
                     'python3 src/reclaim_arm_moveit_config/scripts/pick_and_place.py '
                     '--listen --port 8081'],
                output='screen',
                name='pickup_listener',
            ),
        ],
    )

    # ── Build launch description ───────────────────────────────
    return LaunchDescription([
        trt_model_arg,
        trt_conf_arg,
        trt_port_arg,
        serial_perms,
        teensy_bridge,
        move_group,
        trt_server,
        camera_bridge,
        foxglove_bridge,
        planning_scene,
        pickup_listener,
    ])
