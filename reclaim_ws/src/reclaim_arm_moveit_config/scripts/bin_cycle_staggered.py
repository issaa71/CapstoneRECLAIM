#!/usr/bin/env python3
"""
bin_cycle_staggered.py — Same as bin_cycle but moves J1 first, then J2/J3/J4.
Tests if staggered servo movements reduce spazzing (power brownout).
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


JOINT_NAMES = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint']


def go_to_joints(node, client, joints, label, vel=0.6, accel=0.1):
    print(f"Moving to {label}: {[f'{j:.3f}' for j in joints]}")

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
    for name, value in zip(JOINT_NAMES, joints):
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


def staggered_move(node, client, current_joints, target_joints, label, vel=0.6, accel=0.1):
    """Move to target in three steps: J1+J2+J3 together, then J4 separately."""
    # Step 1: Move J1+J2+J3 (keep J4 at current)
    j123_changed = (abs(target_joints[0] - current_joints[0]) > 0.05 or
                    abs(target_joints[1] - current_joints[1]) > 0.05 or
                    abs(target_joints[2] - current_joints[2]) > 0.05)
    if j123_changed:
        intermediate = [target_joints[0], target_joints[1], target_joints[2], current_joints[3]]
        print(f"  Step 1: Move J1+J2+J3")
        if not go_to_joints(node, client, intermediate, f"{label} (J1-J3)", vel, accel):
            return False, current_joints
        time.sleep(0.5)
        current_joints = list(intermediate)

    # Step 2: Move J4 alone
    if abs(target_joints[3] - current_joints[3]) > 0.05:
        print(f"  Step 2: Move J4")
        if not go_to_joints(node, client, target_joints, f"{label} (J4)", vel, accel):
            return False, current_joints
        time.sleep(0.5)

    return True, list(target_joints)


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
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'config', 'pick_and_place.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    poses = config['poses']

    rclpy.init()
    node = rclpy.create_node('bin_cycle_staggered')

    client = ActionClient(node, MoveGroup, '/move_action')
    print("Waiting for MoveGroup...")
    client.wait_for_server()
    print("Connected.")

    pub = node.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
    time.sleep(0.5)

    print("\n=== STAGGERED BIN CYCLE TEST ===\n")

    current = list(poses['home'])

    # Home + open gripper
    go_to_joints(node, client, poses['home'], 'HOME')
    open_gripper(node, pub)

    # Close → Bin 1 (staggered) → open
    set_gripper(node, pub, 0.7, 'close')
    ok, current = staggered_move(node, client, current, poses['pre_bin_recyclable'], 'BIN 1 (recyclable)')
    open_gripper(node, pub)

    # Home → close → Bin 2 (staggered) → open
    ok, current = staggered_move(node, client, current, poses['home'], 'HOME')
    set_gripper(node, pub, 0.7, 'close')
    ok, current = staggered_move(node, client, current, poses['pre_bin_compost'], 'BIN 2 (compost)')
    open_gripper(node, pub)

    # Home → close → Bin 3 (staggered) → open
    ok, current = staggered_move(node, client, current, poses['home'], 'HOME')
    set_gripper(node, pub, 0.7, 'close')
    ok, current = staggered_move(node, client, current, poses['pre_bin_landfill'], 'BIN 3 (landfill)')
    open_gripper(node, pub)

    # Home
    ok, current = staggered_move(node, client, current, poses['home'], 'HOME')

    print("\n=== STAGGERED BIN CYCLE COMPLETE ===")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
