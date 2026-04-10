#!/usr/bin/env python3
"""
bin_cycle.py — Cycle through all 3 bins from home position.
Home(open) → close → bin1(open) → home(close) → bin2(open) → home(close) → bin3(open) → home
"""

import time
import yaml
import os
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    RobotState, PlanningOptions,
)


def go_to_joints(node, client, joints, label, vel=0.6, accel=0.1):
    joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint']
    print(f"Moving to {label}...")

    goal_msg = MoveGroup.Goal()
    goal_msg.planning_options = PlanningOptions()
    goal_msg.planning_options.plan_only = False
    goal_msg.planning_options.replan = True

    request = MotionPlanRequest()
    request.group_name = 'arm'
    request.num_planning_attempts = 10
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = vel
    request.max_acceleration_scaling_factor = accel
    request.start_state = RobotState()
    request.start_state.is_diff = True

    constraints = Constraints()
    for name, value in zip(joint_names, joints):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = value
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    request.goal_constraints.append(constraints)
    goal_msg.request = request

    future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    gh = future.result()
    if not gh or not gh.accepted:
        print(f"  REJECTED")
        return False

    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
    result = result_future.result()
    if result and result.status == 4 and result.result.error_code.val == 1:
        print(f"  Reached {label}")
        return True
    else:
        print(f"  FAILED {label}")
        return False


def set_gripper(node, pub, val, label):
    print(f"Gripper: {label} ({val} rad)")
    msg = Float64MultiArray()
    steps = 5
    for s in range(1, steps + 1):
        v = val * s / steps
        msg.data = [v]
        pub.publish(msg)
        time.sleep(0.15)
    time.sleep(1.0)


def open_gripper(node, pub):
    print("Gripper: opening")
    msg = Float64MultiArray()
    msg.data = [0.0]
    pub.publish(msg)
    time.sleep(1.3)


def main():
    # Load config
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'config', 'pick_and_place.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    poses = config['poses']

    rclpy.init()
    node = rclpy.create_node('bin_cycle')

    client = ActionClient(node, MoveGroup, '/move_action')
    print("Waiting for MoveGroup...")
    client.wait_for_server()
    print("Connected.")

    pub = node.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
    time.sleep(0.5)

    print("\n=== BIN CYCLE TEST ===\n")

    # Home + open gripper
    go_to_joints(node, client, poses['home'], 'HOME')
    open_gripper(node, pub)

    # Close gripper → Bin 1 (recyclable) → open
    set_gripper(node, pub, 0.7, 'close')
    go_to_joints(node, client, poses['pre_bin_recyclable'], 'BIN 1 (recyclable)')
    open_gripper(node, pub)

    # Home → close → Bin 2 (compost) → open
    go_to_joints(node, client, poses['home'], 'HOME')
    set_gripper(node, pub, 0.7, 'close')
    go_to_joints(node, client, poses['pre_bin_compost'], 'BIN 2 (compost)')
    open_gripper(node, pub)

    # Home → close → Bin 3 (landfill) → open
    go_to_joints(node, client, poses['home'], 'HOME')
    set_gripper(node, pub, 0.7, 'close')
    go_to_joints(node, client, poses['pre_bin_landfill'], 'BIN 3 (landfill)')
    open_gripper(node, pub)

    # Home
    go_to_joints(node, client, poses['home'], 'HOME')

    print("\n=== BIN CYCLE COMPLETE ===")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
