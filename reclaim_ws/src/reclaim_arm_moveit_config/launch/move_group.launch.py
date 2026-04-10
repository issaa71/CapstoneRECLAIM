import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # ── File paths (direct, not from installed packages) ──
    config_dir = os.path.expanduser(
        '~/reclaim_ws/src/reclaim_arm_moveit_config/config')
    urdf_file = os.path.expanduser(
        '~/reclaim_ws/src/reclaim_control/urdf/reclaim_arm.urdf')
    srdf_file = os.path.expanduser(
        '~/reclaim_ws/src/reclaim_arm_moveit_config/srdf/reclaim_arm.srdf')
    ros2_controllers_file = os.path.join(config_dir, 'ros2_controllers.yaml')

    # ── Read URDF (contains <ros2_control> block for hardware interface) ──
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── Read SRDF ──
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # ── Load MoveIt2 config YAMLs ──
    with open(os.path.join(config_dir, 'kinematics.yaml'), 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    with open(os.path.join(config_dir, 'ompl_planning.yaml'), 'r') as f:
        ompl_planning_yaml = yaml.safe_load(f)

    with open(os.path.join(config_dir, 'joint_limits.yaml'), 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)

    with open(os.path.join(config_dir, 'moveit_controllers.yaml'), 'r') as f:
        moveit_controllers_yaml = yaml.safe_load(f)

    # ─────────────────────────────────────────────────────────────────
    # 1. Robot State Publisher — publishes URDF to /robot_description
    #    and broadcasts TF from /joint_states
    # ─────────────────────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ─────────────────────────────────────────────────────────────────
    # 2. ros2_control Controller Manager — loads ServoHardwareInterface
    #    plugin from the <ros2_control> block in URDF, opens serial port,
    #    sends ARM command, runs read()/write() at 50 Hz
    # ─────────────────────────────────────────────────────────────────
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            ros2_controllers_file,
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # 3. Controller Spawners — activate controllers after controller
    #    manager is ready. 30s timeout handles Teensy boot + ARM delay.
    #    Delayed 5s to let controller_manager fully initialize
    #    (open serial, wait for Teensy boot, send ARM = ~4s total)
    # ─────────────────────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # 4. MoveIt2 move_group — motion planning + execution
    #    Uses moveit_simple_controller_manager to talk to arm_controller
    #    via FollowJointTrajectory action
    # ─────────────────────────────────────────────────────────────────
    # MoveIt2 creates a PlanningPipeline per entry in 'planning_pipelines'.
    # Each pipeline looks for <pipeline_name>.planning_plugin, etc.
    # Without 'planning_pipelines', it falls back to using the node name
    # ('move_group') as the pipeline namespace and picks CHOMP if available.
    # Fix: explicitly name the pipeline 'ompl' and pass config under 'ompl.*'.
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_yaml,
                'robot_description_planning': ompl_planning_yaml,
                # Tell MoveIt2 to use the 'ompl' planning pipeline
                'planning_pipelines': ['ompl'],
                'default_planning_pipeline': 'ompl',
                # Nest OMPL config under 'ompl.' namespace so the pipeline
                # finds ompl.planning_plugin, ompl.request_adapters, etc.
                'ompl': ompl_planning_yaml,
                'planning_scene_monitor_options': {
                    'joint_state_topic': '/joint_states',
                    'attached_collision_object_topic': '/planning_scene',
                    'publish_planning_scene_topic': '/publish_planning_scene',
                    'monitored_planning_scene_topic': '/monitored_planning_scene',
                    'wait_for_initial_state_timeout': 10.0,
                },
                # MoveIt2 sends FollowJointTrajectory goals to arm_controller
                # (ros2_control's JointTrajectoryController provides the action server)
                'moveit_controller_manager':
                    'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            },
            moveit_controllers_yaml,
            joint_limits_yaml,
        ],
    )

    # ─────────────────────────────────────────────────────────────────
    # 5. RViz2 with MoveIt2 MotionPlanning plugin
    # ─────────────────────────────────────────────────────────────────
    rviz_config = os.path.expanduser(
        '~/reclaim_ws/src/reclaim_arm_moveit_config/config/moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'robot_description_kinematics': kinematics_yaml},
        ],
    )

    # ─────────────────────────────────────────────────────────────────
    # 6. Static transform: world → base_link (identity)
    #    Required because the SRDF virtual_joint defines world as the
    #    parent frame, but nothing publishes this TF automatically.
    # ─────────────────────────────────────────────────────────────────
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
        output='screen',
    )

    # ─────────────────────────────────────────────────────────────────
    # Launch order with delays:
    #   t=0s: robot_state_publisher + controller_manager + rviz + static TF
    #   t=5s: spawners (controller_manager needs ~4s for serial + ARM)
    #   t=8s: move_group (after controllers are active)
    # ─────────────────────────────────────────────────────────────────
    delayed_spawners = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    delayed_move_group = TimerAction(
        period=8.0,
        actions=[move_group_node],
    )

    return LaunchDescription([
        static_tf_node,
        robot_state_publisher_node,
        controller_manager_node,
        rviz_node,
        delayed_spawners,
        delayed_move_group,
    ])
