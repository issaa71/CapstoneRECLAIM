#!/usr/bin/env python3
"""
goto_xyz.py — Send tool0 to an XYZ position using MoveIt2 PositionConstraint.
Visualize in RViz. Same approach as pick_and_place camera mode.

Usage (with move_group running):
  python3 goto_xyz.py 200 0 -100              # move only
  python3 goto_xyz.py 200 0 -100 --grip 0.7   # move then close gripper to 0.7 rad
  python3 goto_xyz.py 200 0 -100 --grip 0.0   # move then open gripper
"""

import sys
import time
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    RobotState,
    PlanningOptions,
)
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


def main():
    if len(sys.argv) < 4:
        print("Usage: python3 goto_xyz.py X_mm Y_mm Z_mm [--grip VAL] [--vel V] [--accel A]")
        print("  X_mm, Y_mm, Z_mm: position relative to J1 in millimeters")
        print("  --grip VAL: close gripper to VAL rad after reaching position (0.0=open, 0.7=closed)")
        print("  --vel V: velocity scaling 0-1 (default 0.6)")
        print("  --accel A: acceleration scaling 0-1 (default 0.1)")
        print("\nExamples:")
        print("  python3 goto_xyz.py 200 0 -100")
        print("  python3 goto_xyz.py 200 0 -100 --grip 0.7")
        sys.exit(1)

    x_mm = float(sys.argv[1])
    y_mm = float(sys.argv[2])
    z_mm = float(sys.argv[3])

    # Parse optional args
    grip_val = None
    vel = 0.6
    accel = 0.1
    i = 4
    while i < len(sys.argv):
        if sys.argv[i] == '--grip' and i + 1 < len(sys.argv):
            grip_val = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--vel' and i + 1 < len(sys.argv):
            vel = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--accel' and i + 1 < len(sys.argv):
            accel = float(sys.argv[i + 1])
            i += 2
        else:
            i += 1

    x = x_mm / 1000.0
    y = y_mm / 1000.0
    z = z_mm / 1000.0

    print(f"Target: X={x_mm:.0f} Y={y_mm:.0f} Z={z_mm:.0f} mm")
    print(f"Velocity: {vel}, Acceleration: {accel}")

    rclpy.init()
    node = rclpy.create_node('goto_xyz')

    client = ActionClient(node, MoveGroup, '/move_action')
    print("Waiting for MoveGroup action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        print("MoveGroup not available!")
        rclpy.shutdown()
        sys.exit(1)
    print("Connected.")

    # Build MoveGroup goal with PositionConstraint
    goal_msg = MoveGroup.Goal()
    goal_msg.planning_options = PlanningOptions()
    goal_msg.planning_options.plan_only = False
    goal_msg.planning_options.replan = True
    goal_msg.planning_options.replan_attempts = 3

    request = MotionPlanRequest()
    request.group_name = 'arm'
    request.num_planning_attempts = 10
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = vel
    request.max_acceleration_scaling_factor = accel

    request.start_state = RobotState()
    request.start_state.is_diff = True

    # Position constraint — "put tool0 at this XYZ"
    pc = PositionConstraint()
    pc.header.frame_id = 'base_link'
    pc.header.stamp = node.get_clock().now().to_msg()
    pc.link_name = 'tool0'
    pc.weight = 1.0

    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.w = 1.0

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.02]  # 20mm tolerance

    pc.constraint_region.primitives.append(sphere)
    pc.constraint_region.primitive_poses.append(target_pose)

    constraints = Constraints()
    constraints.position_constraints.append(pc)
    request.goal_constraints.append(constraints)

    goal_msg.request = request

    # Send goal
    print(f"Planning to X={x_mm:.0f} Y={y_mm:.0f} Z={z_mm:.0f} mm...")
    future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    goal_handle = future.result()
    if not goal_handle or not goal_handle.accepted:
        print("Goal rejected!")
        rclpy.shutdown()
        sys.exit(1)

    print("Executing...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)

    result = result_future.result()
    if result and result.status == 4:
        error_code = result.result.error_code.val
        if error_code == 1:
            print(f"SUCCESS — tool0 at X={x_mm:.0f} Y={y_mm:.0f} Z={z_mm:.0f} mm")

            # Gripper command if requested
            if grip_val is not None:
                gripper_pub = node.create_publisher(
                    Float64MultiArray, '/gripper_controller/commands', 10)
                time.sleep(0.3)  # let publisher connect
                msg = Float64MultiArray()
                # Slow close in steps
                steps = 5
                for s in range(1, steps + 1):
                    val = grip_val * s / steps
                    msg.data = [val]
                    gripper_pub.publish(msg)
                    time.sleep(0.15)
                time.sleep(1.0)  # extra settle time
                print(f"Gripper set to {grip_val} rad")
        else:
            print(f"FAILED — error code: {error_code}")
    else:
        status = result.status if result else 'timeout'
        print(f"FAILED — status: {status}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
