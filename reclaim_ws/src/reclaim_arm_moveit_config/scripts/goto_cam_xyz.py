#!/usr/bin/env python3
"""
goto_cam_xyz.py — Send tool0 to a position given in CAMERA coordinates.
Transforms camera XYZ → base_link XYZ via TF, then moves arm there.
Tests the full camera-to-arm transform chain without needing the camera running.

Usage (with move_group running, arm in look-down/home pose):
  python3 goto_cam_xyz.py -50 2 235              # camera X=-50mm Y=2mm Z=235mm (depth)
  python3 goto_cam_xyz.py -50 2 235 --grip 0.7   # move then close gripper
"""

import sys
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Pose
from std_msgs.msg import Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    RobotState,
    PlanningOptions,
)
from shape_msgs.msg import SolidPrimitive


def main():
    if len(sys.argv) < 4:
        print("Usage: python3 goto_cam_xyz.py cam_X_mm cam_Y_mm cam_Z_mm [--grip VAL]")
        print("  cam_X/Y/Z: position in camera optical frame (mm)")
        print("    X = right in image, Y = down in image, Z = depth (forward)")
        print("  --grip VAL: close gripper after reaching (0.0=open, 0.7=closed)")
        print("\nExample: python3 goto_cam_xyz.py -50 2 235 --grip 0.7")
        sys.exit(1)

    cam_x_mm = float(sys.argv[1])
    cam_y_mm = float(sys.argv[2])
    cam_z_mm = float(sys.argv[3])

    grip_val = None
    i = 4
    while i < len(sys.argv):
        if sys.argv[i] == '--grip' and i + 1 < len(sys.argv):
            grip_val = float(sys.argv[i + 1])
            i += 2
        else:
            i += 1

    print(f"Camera frame: X={cam_x_mm:.0f} Y={cam_y_mm:.0f} Z={cam_z_mm:.0f} mm")

    rclpy.init()
    node = rclpy.create_node('goto_cam_xyz')

    # Setup TF listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    # Wait for TF to be available
    print("Waiting for TF: base_link → camera_optical_frame ...")
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.5)
        try:
            tf_buffer.lookup_transform('base_link', 'camera_optical_frame',
                                        rclpy.time.Time(), timeout=Duration(seconds=0.5))
            print("TF available!")
            break
        except Exception:
            pass
    else:
        print("TF not available! Is move_group running?")
        rclpy.shutdown()
        sys.exit(1)

    # Spin a bit more to get fresh TF
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Print all intermediate transforms for debugging
    print("\n=== TRANSFORM CHAIN DEBUG ===")
    print(f"Step 1 — Camera optical frame input:")
    print(f"  cam_x={cam_x_mm:.0f}mm (right), cam_y={cam_y_mm:.0f}mm (down), cam_z={cam_z_mm:.0f}mm (depth)")

    # Step 2: Get optical → camera_link transform
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
        tf_opt_to_link = tf_buffer.lookup_transform(
            'camera_link', 'camera_optical_frame',
            rclpy.time.Time(), timeout=Duration(seconds=1.0))
        r = tf_opt_to_link.transform.rotation
        t = tf_opt_to_link.transform.translation
        print(f"\nStep 2 — optical→camera_link transform:")
        print(f"  Translation: ({t.x*1000:.1f}, {t.y*1000:.1f}, {t.z*1000:.1f}) mm")
        print(f"  Rotation quat: ({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})")
    except Exception as e:
        print(f"  Could not get optical→link transform: {e}")

    # Step 3: Transform to camera_link
    pt_optical = PointStamped()
    pt_optical.header.frame_id = 'camera_optical_frame'
    pt_optical.header.stamp = rclpy.time.Time().to_msg()
    pt_optical.point.x = cam_x_mm / 1000.0
    pt_optical.point.y = cam_y_mm / 1000.0
    pt_optical.point.z = cam_z_mm / 1000.0

    try:
        rclpy.spin_once(node, timeout_sec=0.1)
        link_pt = tf_buffer.transform(pt_optical, 'camera_link', timeout=Duration(seconds=1.0))
        print(f"\nStep 3 — In camera_link frame:")
        print(f"  x_link={link_pt.point.x*1000:.0f}mm, y_link={link_pt.point.y*1000:.0f}mm, z_link={link_pt.point.z*1000:.0f}mm")
        print(f"  (z_link should ≈ depth={cam_z_mm:.0f}mm if Z=forward)")
    except Exception as e:
        print(f"  Could not transform to camera_link: {e}")

    # Step 4: Get camera_link → base_link transform
    try:
        tf_link_to_base = tf_buffer.lookup_transform(
            'base_link', 'camera_link',
            rclpy.time.Time(), timeout=Duration(seconds=1.0))
        r = tf_link_to_base.transform.rotation
        t = tf_link_to_base.transform.translation
        print(f"\nStep 4 — camera_link→base_link transform:")
        print(f"  Translation: ({t.x*1000:.1f}, {t.y*1000:.1f}, {t.z*1000:.1f}) mm")
        print(f"  Rotation quat: ({r.x:.3f}, {r.y:.3f}, {r.z:.3f}, {r.w:.3f})")
    except Exception as e:
        print(f"  Could not get link→base transform: {e}")

    # Step 5: Full transform to base_link
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
        base_pt = tf_buffer.transform(pt_optical, 'base_link', timeout=Duration(seconds=1.0))
        bx = base_pt.point.x
        by = base_pt.point.y
        bz = base_pt.point.z
        print(f"\nStep 5 — In base_link frame:")
        print(f"  x_base={bx*1000:.0f}mm, y_base={by*1000:.0f}mm, z_base={bz*1000:.0f}mm")
        print(f"  Distance from J1: {math.sqrt(bx**2 + by**2)*1000:.0f}mm")
        print(f"=== END DEBUG ===\n")
    except Exception as e:
        print(f"TF transform failed: {e}")
        rclpy.shutdown()
        sys.exit(1)

    # Connect to MoveGroup
    client = ActionClient(node, MoveGroup, '/move_action')
    print("Waiting for MoveGroup...")
    if not client.wait_for_server(timeout_sec=10.0):
        print("MoveGroup not available!")
        rclpy.shutdown()
        sys.exit(1)

    # Build PositionConstraint goal
    goal_msg = MoveGroup.Goal()
    goal_msg.planning_options = PlanningOptions()
    goal_msg.planning_options.plan_only = False
    goal_msg.planning_options.replan = True
    goal_msg.planning_options.replan_attempts = 3

    request = MotionPlanRequest()
    request.group_name = 'arm'
    request.num_planning_attempts = 10
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = 0.6
    request.max_acceleration_scaling_factor = 0.1
    request.start_state = RobotState()
    request.start_state.is_diff = True

    pc = PositionConstraint()
    pc.header.frame_id = 'base_link'
    pc.header.stamp = node.get_clock().now().to_msg()
    pc.link_name = 'tool0'
    pc.weight = 1.0

    target_pose = Pose()
    target_pose.position.x = bx
    target_pose.position.y = by
    target_pose.position.z = bz
    target_pose.orientation.w = 1.0

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.02]

    pc.constraint_region.primitives.append(sphere)
    pc.constraint_region.primitive_poses.append(target_pose)

    constraints = Constraints()
    constraints.position_constraints.append(pc)
    request.goal_constraints.append(constraints)
    goal_msg.request = request

    print(f"Planning to base_link: X={bx*1000:.0f} Y={by*1000:.0f} Z={bz*1000:.0f} mm...")
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
            print(f"SUCCESS")
            print(f"  Camera input: X={cam_x_mm:.0f} Y={cam_y_mm:.0f} Z={cam_z_mm:.0f} mm")
            print(f"  Base_link:    X={bx*1000:.0f} Y={by*1000:.0f} Z={bz*1000:.0f} mm")
            print("  Measure actual tool0 position with ruler and compare!")

            if grip_val is not None:
                gripper_pub = node.create_publisher(
                    Float64MultiArray, '/gripper_controller/commands', 10)
                time.sleep(0.3)
                msg = Float64MultiArray()
                steps = 5
                for s in range(1, steps + 1):
                    val = grip_val * s / steps
                    msg.data = [val]
                    gripper_pub.publish(msg)
                    time.sleep(0.15)
                time.sleep(1.0)
                print(f"Gripper set to {grip_val} rad")
        else:
            print(f"FAILED — error code: {error_code}")
    else:
        status = result.status if result else 'timeout'
        print(f"FAILED — status: {status}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
