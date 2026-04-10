#!/usr/bin/env python3
"""
pick_and_place.py — Pick-and-Place for RECLAIM

Uses MoveGroup action (MoveIt2 OMPL planning with collision avoidance)
+ /compute_ik service (position-only IK with KDL) for arm motions,
and gripper controller topic for gripper.

Modes:
  --test     : Mechanical test — cycle through hardcoded poses, no camera
  --camera   : Full pipeline — OAK-D detection + IK pickup + bin deposit
  --bin NAME : Override bin selection (recyclable, compost, landfill)

Usage (with move_group already running):
  python3 pick_and_place.py --test
  python3 pick_and_place.py --test --bin recyclable
  python3 pick_and_place.py --camera
"""

import argparse
import math
import time
import yaml
import os
import threading
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    RobotState,
    PlanningOptions,
)
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped


# ── MJPEG Stream ────────────────────────────────────────────────

latest_frame = None
frame_lock = threading.Lock()

class MJPEGHandler:
    """Simple MJPEG HTTP handler (no BaseHTTPRequestHandler to avoid import issues)."""
    pass

def start_mjpeg_server(port=8081):
    """Start MJPEG stream server in background thread."""
    from http.server import HTTPServer, BaseHTTPRequestHandler

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                html = '<html><body style="margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh;"><img src="/stream" style="max-width:100%;"></body></html>'
                self.wfile.write(html.encode())
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with frame_lock:
                            frame = latest_frame
                        if frame is not None:
                            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                            self.wfile.write(b'--frame\r\n')
                            self.wfile.write(b'Content-Type: image/jpeg\r\n')
                            self.wfile.write(f'Content-Length: {len(jpeg)}\r\n'.encode())
                            self.wfile.write(b'\r\n')
                            self.wfile.write(jpeg.tobytes())
                            self.wfile.write(b'\r\n')
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
        def log_message(self, format, *args):
            pass  # Suppress HTTP logs

    server = HTTPServer(('0.0.0.0', port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"MJPEG stream on http://0.0.0.0:{port}")


# ── Arm Controller ──────────────────────────────────────────────

class ArmController:
    """Controls arm via MoveGroup action (MoveIt2 OMPL planning) + gripper topic."""

    JOINT_NAMES = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint']
    GROUP_NAME = 'arm'

    def __init__(self, node, config):
        self.node = node
        self.config = config
        self.logger = node.get_logger()

        pickup_cfg = config.get('pickup', {})
        self.max_vel = pickup_cfg.get('max_velocity_scaling', 0.5)
        self.max_accel = pickup_cfg.get('max_acceleration_scaling', 0.1)
        self.planning_time = pickup_cfg.get('planning_time', 5.0)
        self.num_attempts = pickup_cfg.get('num_planning_attempts', 10)

        # MoveGroup action client
        self.move_action_client = ActionClient(
            node, MoveGroup, '/move_action')

        self.logger.info("Waiting for MoveGroup action server (/move_action)...")
        if not self.move_action_client.wait_for_server(timeout_sec=15.0):
            self.logger.error("MoveGroup action server not available!")
            raise RuntimeError("MoveGroup action server not available")
        self.logger.info("MoveGroup connected.")

        # IK service client
        self.ik_client = node.create_client(GetPositionIK, '/compute_ik')
        self.logger.info("Waiting for /compute_ik service...")
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.logger.warn("/compute_ik service not available — go_to_position will fail")
        else:
            self.logger.info("/compute_ik service connected.")

        # Gripper publisher
        self.gripper_pub = node.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        # Load config
        self.poses = config.get('poses', {})
        self.gripper_cfg = config.get('gripper', {})

    def go_to_joints(self, target_joints, label=""):
        """Plan and execute to target joint positions using MoveIt2."""
        self.logger.info(f"Planning to {label}: {[f'{j:.3f}' for j in target_joints]}")

        goal_msg = MoveGroup.Goal()
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        request = MotionPlanRequest()
        request.group_name = self.GROUP_NAME
        request.num_planning_attempts = self.num_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_vel
        request.max_acceleration_scaling_factor = self.max_accel

        request.start_state = RobotState()
        request.start_state.is_diff = True

        constraints = Constraints()
        for name, value in zip(self.JOINT_NAMES, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
        goal_msg.request = request

        future = self.move_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.logger.error(f"MoveGroup goal rejected for '{label}'")
            return False

        self.logger.info(f"Planning + executing '{label}'...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result and result.status == 4:
            error_code = result.result.error_code.val
            if error_code == 1:
                self.logger.info(f"Reached '{label}' successfully")
                return True
            else:
                self.logger.error(f"MoveGroup '{label}' error code: {error_code}")
                return False
        else:
            status = result.status if result else 'timeout'
            self.logger.error(f"MoveGroup '{label}' failed (status: {status})")
            return False

    def solve_ik(self, x, y, z):
        """Solve position-only IK via /compute_ik service.
        Returns joint angles [J1, J2, J3, J4] or None on failure.
        Uses position_only_ik: true in kinematics.yaml (orientation ignored).
        """
        self.logger.info(f"Solving IK for X={x*1000:.0f} Y={y*1000:.0f} Z={z*1000:.0f} mm")

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.GROUP_NAME
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.w = 1.0  # ignored with position_only_ik
        request.ik_request.timeout.sec = 2
        request.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        result = future.result()
        if result is None:
            self.logger.error("IK service call failed (no response)")
            return None

        if result.error_code.val != 1:  # MoveItErrorCodes.SUCCESS = 1
            self.logger.error(f"IK failed with error code: {result.error_code.val}")
            return None

        # Extract joint angles for our 4 joints
        solution = result.solution.joint_state
        joint_angles = []
        for name in self.JOINT_NAMES:
            if name in solution.name:
                idx = list(solution.name).index(name)
                joint_angles.append(solution.position[idx])
            else:
                self.logger.error(f"Joint {name} not in IK solution")
                return None

        self.logger.info(f"IK solution: {[f'{j:.3f}' for j in joint_angles]}")
        return joint_angles

    def go_to_position(self, x, y, z, label="position"):
        """Move tool0 to target XYZ using PositionConstraint.
        MoveIt2 handles IK + trajectory planning together — same as RViz internally.
        No separate IK call, no J4 override. MoveIt2 finds the best solution."""
        self.logger.info(f"Moving to {label}: X={x*1000:.0f} Y={y*1000:.0f} Z={z*1000:.0f} mm")

        from moveit_msgs.msg import PositionConstraint
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose

        goal_msg = MoveGroup.Goal()
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        request = MotionPlanRequest()
        request.group_name = self.GROUP_NAME
        request.num_planning_attempts = self.num_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_vel
        request.max_acceleration_scaling_factor = self.max_accel

        request.start_state = RobotState()
        request.start_state.is_diff = True

        # Position constraint — tell MoveIt2 "put tool0 here"
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.header.stamp = self.node.get_clock().now().to_msg()
        pc.link_name = 'tool0'
        pc.weight = 1.0

        # Target position as a small sphere (tolerance region)
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]  # 20mm tolerance sphere

        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(target_pose)

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        request.goal_constraints.append(constraints)

        goal_msg.request = request

        # Send to MoveGroup
        future = self.move_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.logger.error(f"MoveGroup goal rejected for '{label}'")
            return False

        self.logger.info(f"Planning + executing '{label}'...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result and result.status == 4:
            error_code = result.result.error_code.val
            if error_code == 1:
                self.logger.info(f"Reached '{label}' successfully")
                return True
            else:
                self.logger.error(f"MoveGroup '{label}' error code: {error_code}")
                return False
        else:
            status = result.status if result else 'timeout'
            self.logger.error(f"MoveGroup '{label}' failed (status: {status})")
            return False

    def go_to_pose(self, pose_name):
        """Move arm to a named pose from config."""
        if pose_name not in self.poses:
            self.logger.error(f"Unknown pose: {pose_name}")
            return False
        return self.go_to_joints(self.poses[pose_name], label=pose_name)

    def open_gripper(self):
        val = self.gripper_cfg.get('open', 0.0)
        self.logger.info(f"Opening gripper ({val} rad)")
        msg = Float64MultiArray()
        msg.data = [val]
        self.gripper_pub.publish(msg)
        time.sleep(self.gripper_cfg.get('open_delay', 0.3))
        time.sleep(1.0)  # extra settle time

    def close_gripper(self, class_name=None):
        # Use per-class grip strength if available
        class_grip = self.gripper_cfg.get('class_grip', {})
        if class_name and class_name in class_grip:
            target = class_grip[class_name]
        else:
            target = self.gripper_cfg.get('close', 0.5)
        steps = self.gripper_cfg.get('close_steps', 5)
        self.logger.info(f"Closing gripper slowly ({target} rad, {steps} steps, class={class_name})")
        msg = Float64MultiArray()
        for i in range(1, steps + 1):
            val = target * i / steps
            msg.data = [val]
            self.gripper_pub.publish(msg)
            time.sleep(0.15)
        time.sleep(self.gripper_cfg.get('close_delay', 0.3))
        time.sleep(1.0)  # extra settle time


# ── Camera Detection ───────────────────────────────────────────

def detect_object_once(model, rgb_queue, depth_queue, intrinsics,
                       conf_threshold=0.35, burst_count=5):
    """
    Run one detection cycle: get frame, run YOLO, get depth via burst median.
    Returns: (x_base, y_base, z_base, class_name, confidence) or None
    """
    import numpy as np
    import cv2
    import tf2_geometry_msgs
    from geometry_msgs.msg import PointStamped
    from rclpy.duration import Duration

    global latest_frame

    # Get RGB frame
    rgb_data = rgb_queue.tryGet()
    if rgb_data is None:
        return None
    frame = rgb_data.getCvFrame()
    annotated = frame.copy()

    # Run YOLO with confidence threshold
    results = model(frame, conf=conf_threshold, verbose=False)
    if len(results) == 0 or len(results[0].boxes) == 0:
        cv2.putText(annotated, "No objects detected", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        with frame_lock:
            latest_frame = annotated
        return None

    # Get best detection (highest confidence)
    boxes = results[0].boxes
    best_idx = boxes.conf.argmax().item()
    box = boxes[best_idx]
    conf = float(box.conf[0])
    cls_id = int(box.cls[0])
    class_name = model.names[cls_id]

    # Get bbox center
    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)

    # Draw bbox on annotated frame
    cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
    cv2.putText(annotated, f"{class_name} {conf:.2f}", (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.drawMarker(annotated, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 15, 2)

    # Burst depth with 3D bounding box analysis
    fx, fy, cx_intr, cy_intr = intrinsics
    obj_depths = []     # depth at object center (inner bbox)
    ground_depths = []  # depth at bbox edges (ground around object)
    for i in range(burst_count):
        depth_data = depth_queue.tryGet()
        if depth_data is None:
            time.sleep(0.1)
            depth_data = depth_queue.tryGet()
            if depth_data is None:
                continue
        depth_frame = depth_data.getCvFrame()

        # Full bbox region
        y_lo_full = max(0, int(y1))
        y_hi_full = min(depth_frame.shape[0], int(y2))
        x_lo_full = max(0, int(x1))
        x_hi_full = min(depth_frame.shape[1], int(x2))
        full_roi = depth_frame[y_lo_full:y_hi_full, x_lo_full:x_hi_full]

        # Inner region (object surface) — center 40%
        bw = max(1, int((x2 - x1) * 0.2))
        bh = max(1, int((y2 - y1) * 0.2))
        y_lo = max(0, cy - bh)
        y_hi = min(depth_frame.shape[0], cy + bh)
        x_lo = max(0, cx - bw)
        x_hi = min(depth_frame.shape[1], cx + bw)
        inner_roi = depth_frame[y_lo:y_hi, x_lo:x_hi]

        valid_inner = inner_roi[(inner_roi > 150) & (inner_roi < 600)]
        valid_full = full_roi[(full_roi > 150) & (full_roi < 600)]

        if len(valid_inner) > 0:
            obj_depths.append(float(np.median(valid_inner)))
        if len(valid_full) > 0:
            # Ground = max depth in full bbox (furthest from camera = ground)
            ground_depths.append(float(np.percentile(valid_full, 90)))

        if i < burst_count - 1:
            time.sleep(0.05)

    if len(obj_depths) < 2:
        return None

    obj_z_mm = float(np.median(obj_depths))
    ground_z_mm = float(np.median(ground_depths)) if len(ground_depths) >= 2 else obj_z_mm
    object_height_mm = max(0, ground_z_mm - obj_z_mm)

    # Use object center for XY, ground depth for Z (grab at ground level + offset)
    z_mm = obj_z_mm
    x_mm = (cx - cx_intr) * z_mm / fx
    y_mm = (cy - cy_intr) * z_mm / fy

    # Also compute ground point (for grab Z)
    ground_x_mm = (cx - cx_intr) * ground_z_mm / fx
    ground_y_mm = (cy - cy_intr) * ground_z_mm / fy

    # Transform to base_link via TF
    global tf_node, tf_buffer
    if tf_node is None:
        return None

    try:
        rclpy.spin_once(tf_node, timeout_sec=0.1)

        # Transform object center to base_link (for XY position)
        pt = PointStamped()
        pt.header.frame_id = 'camera_optical_frame'
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = x_mm / 1000.0
        pt.point.y = y_mm / 1000.0
        pt.point.z = z_mm / 1000.0

        from rclpy.duration import Duration
        base_pt = tf_buffer.transform(pt, 'base_link', timeout=Duration(seconds=0.5))

        # Transform ground point to base_link (for grab Z)
        pt_ground = PointStamped()
        pt_ground.header.frame_id = 'camera_optical_frame'
        pt_ground.header.stamp = rclpy.time.Time().to_msg()
        pt_ground.point.x = ground_x_mm / 1000.0
        pt_ground.point.y = ground_y_mm / 1000.0
        pt_ground.point.z = ground_z_mm / 1000.0
        base_ground = tf_buffer.transform(pt_ground, 'base_link', timeout=Duration(seconds=0.5))

        # Grab height = ground Z + half the object height (in base_link)
        grab_offset = (object_height_mm / 1000.0) * 0.4  # grab at 40% up from ground

        # Apply detection calibration offsets (from config)
        cal = calibration_offsets
        bx = base_pt.point.x + cal.get('detection_offset_x', 0.0)
        by = base_pt.point.y + cal.get('detection_offset_y', 0.0)
        bz = base_pt.point.z + cal.get('detection_offset_z', 0.0)
        height_source = f"raw Z (h={object_height_mm:.0f}mm)"

        # Annotate frame with base_link coords
        ly = int(y2) + 20
        for txt, color in [
            (f"cam: X={x_mm:.0f} Y={y_mm:.0f} Z={z_mm:.0f}mm", (0, 255, 0)),
            (f"base: X={bx*1000:.0f} Y={by*1000:.0f} Z={bz*1000:.0f}mm", (255, 200, 0)),
            (f"dist: {math.sqrt(bx**2+by**2)*1000:.0f}mm  h={object_height_mm:.0f}mm", (255, 200, 0)),
            (f"grab: {height_source}", (0, 200, 255)),
        ]:
            cv2.putText(annotated, txt, (int(x1), ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(annotated, txt, (int(x1), ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            ly += 20

        with frame_lock:
            latest_frame = annotated

        return (bx, by, bz, class_name, conf)
    except Exception as e:
        print(f"TF error: {e}")
        with frame_lock:
            latest_frame = annotated
        return None


# TF globals
tf_node = None
tf_buffer = None
tf_listener = None  # Must store to prevent GC
calibration_offsets = {}


def init_tf():
    """Initialize TF2 listener."""
    global tf_node, tf_buffer, tf_listener
    import tf2_ros
    tf_node = rclpy.create_node('pick_place_tf')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, tf_node)  # stored globally

    from rclpy.duration import Duration
    for _ in range(20):
        rclpy.spin_once(tf_node, timeout_sec=0.5)
        try:
            tf_buffer.lookup_transform('base_link', 'camera_optical_frame',
                                       rclpy.time.Time(), timeout=Duration(seconds=0.5))
            print("TF available!")
            return True
        except Exception:
            pass
    print("TF not available!")
    return False


# ── Test Mode ───────────────────────────────────────────────────

def run_test_mode(arm, bin_name=None):
    """Cycle through poses to test arm motions + gripper."""
    logger = arm.node.get_logger()
    logger.info("=== TEST MODE: Cycling through poses ===")

    if not arm.go_to_pose('home'):
        logger.error("Failed to reach home. Aborting.")
        return

    arm.open_gripper()
    time.sleep(0.5)

    arm.close_gripper()
    time.sleep(0.5)

    arm.go_to_pose('home')

    if bin_name:
        pose_name = f'pre_bin_{bin_name}'
        if pose_name in arm.poses:
            arm.go_to_pose(pose_name)
            arm.open_gripper()
            time.sleep(0.5)
            arm.go_to_pose('home')
        else:
            logger.error(f"Bin pose '{pose_name}' not found in config")
    else:
        for bn in ['recyclable', 'compost', 'landfill']:
            pose_name = f'pre_bin_{bn}'
            if pose_name in arm.poses:
                logger.info(f"--- Going to {bn} bin ---")
                arm.go_to_pose(pose_name)
                arm.open_gripper()
                time.sleep(0.5)

    arm.close_gripper()
    arm.go_to_pose('home')
    logger.info("=== TEST COMPLETE ===")


# ── Camera Mode ─────────────────────────────────────────────────

def run_camera_mode(arm, config, model_path, conf_threshold):
    """Full camera pickup pipeline: detect → pick → sort."""
    import depthai as dai
    import numpy as np

    global calibration_offsets
    calibration_offsets = config.get('calibration', {})

    logger = arm.node.get_logger()
    logger.info("=== CAMERA MODE ===")

    # Init TF
    if not init_tf():
        logger.error("TF not available. Aborting.")
        return

    # Load YOLO model + warmup BEFORE camera (prevents pipeline buffer stall)
    from ultralytics import YOLO
    model = YOLO(model_path)
    logger.info(f"Model loaded: {model_path}")

    # Skip warmup — first real inference will be slow but avoids LD_PRELOAD conflicts

    # Class to bin mapping
    class_to_bin = config.get('class_to_bin', {})

    # Go to home (look-down) position BEFORE starting camera
    arm.go_to_pose('home')
    arm.open_gripper()

    # Camera pipeline — must use 'with' block (DepthAI v3 requirement)
    with dai.Pipeline() as pipeline:
        # RGB camera
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        rgb_queue = cam.requestOutput((640, 480)).createOutputQueue(
            maxSize=1, blocking=False)  # Non-blocking to prevent stall

        # Stereo depth
        stereo = pipeline.create(dai.node.StereoDepth).build(autoCreateCameras=True)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(640, 480)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setExtendedDisparity(True)
        depth_queue = stereo.depth.createOutputQueue(
            maxSize=1, blocking=False)  # Non-blocking

        pipeline.start()
        logger.info("Camera started.")

        # Get intrinsics
        calib = pipeline.getDefaultDevice().readCalibration()
        intrinsics_matrix = np.array(calib.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A, 640, 480))
        intrinsics = (intrinsics_matrix[0][0], intrinsics_matrix[1][1],
                      intrinsics_matrix[0][2], intrinsics_matrix[1][2])
        logger.info(f"Intrinsics: fx={intrinsics[0]:.1f} fy={intrinsics[1]:.1f}")

        # Update TF buffer with current arm pose (critical!)
        # The arm moved to home but tf_node wasn't spun during that motion
        logger.info("Updating TF buffer for current arm pose...")
        for _ in range(30):
            rclpy.spin_once(tf_node, timeout_sec=0.1)

        # Let camera settle + flush stale frames
        time.sleep(2.0)
        for _ in range(10):
            rgb_queue.tryGet()
            depth_queue.tryGet()
            time.sleep(0.05)

        logger.info("Scanning for objects...")

        # Detection loop
        max_attempts = 15
        detection = None
        for attempt in range(max_attempts):
            logger.info(f"  Attempt {attempt + 1}/{max_attempts}...")
            detection = detect_object_once(
                model, rgb_queue, depth_queue, intrinsics,
                conf_threshold=conf_threshold)
            if detection:
                break
            time.sleep(0.3)

        if not detection:
            logger.error("No object detected after scanning. Returning home.")
            arm.go_to_pose('home')
            return

        bx, by, bz, class_name, conf = detection
        logger.info(f"Detected: {class_name} (conf={conf:.2f})")
        logger.info(f"  base_link: X={bx*1000:.0f} Y={by*1000:.0f} Z={bz*1000:.0f} mm")

        # Determine bin
        bin_name = class_to_bin.get(class_name, 'landfill')
        logger.info(f"  Bin: {bin_name}")

    # Camera released — now do the pickup (outside 'with' block)
    logger.info("--- PICKUP SEQUENCE ---")

    # 1. Move directly to pick position — MoveIt2 handles IK + planning (like RViz)
    if not arm.go_to_position(bx, by, bz, label='pick'):
        logger.error("Failed to reach pick position. Aborting.")
        arm.go_to_pose('home')
        return

    # 2. Close gripper (per-class strength)
    arm.close_gripper(class_name=class_name)
    time.sleep(1.0)  # Let servos settle before moving

    # 3. Move to safe waypoint (clear of bins)
    safe_wp = arm.poses.get('safe_waypoint')
    if safe_wp:
        logger.info("Moving to safe waypoint...")
        arm.go_to_joints(safe_wp, label='safe_waypoint')

    # 4. Go to bin
    bin_pose = f'pre_bin_{bin_name}'
    if bin_pose in arm.poses:
        logger.info(f"--- DEPOSITING in {bin_name} bin ---")
        arm.go_to_pose(bin_pose)
        arm.open_gripper()
        time.sleep(0.5)
    else:
        logger.error(f"Bin pose '{bin_pose}' not found. Dropping here.")
        arm.open_gripper()

    # 7. Return home
    arm.go_to_pose('home')
    logger.info("=== PICKUP COMPLETE ===")


# ── TRT Mode (fetches from TRT server) ──────────────────────────

def fetch_detection_from_trt(port, timeout=10.0):
    """Fetch latest detection from TRT server API."""
    import urllib.request
    import json

    url = f'http://localhost:{port}/detection'
    start = time.time()
    while time.time() - start < timeout:
        try:
            resp = urllib.request.urlopen(url, timeout=2)
            data = json.loads(resp.read().decode())
            if data.get('status') == 'no_detection':
                time.sleep(0.2)
                continue
            return data
        except Exception:
            time.sleep(0.3)
    return None


def run_trt_mode(arm, config, port):
    """Pickup pipeline using TRT server for detection (30 FPS)."""
    logger = arm.node.get_logger()
    logger.info("=== TRT MODE (30 FPS) ===")

    calibration = config.get('calibration', {})
    class_to_bin = config.get('class_to_bin', {})
    class_offsets = config.get('class_offsets', {})

    # Go to home
    arm.go_to_pose('home')
    arm.open_gripper()

    # Wait for camera to settle
    time.sleep(2.0)

    # Fetch detection from TRT server
    logger.info(f"Fetching detection from TRT server (port {port})...")
    max_attempts = 15
    detection = None
    for attempt in range(max_attempts):
        logger.info(f"  Attempt {attempt + 1}/{max_attempts}...")
        det = fetch_detection_from_trt(port, timeout=3.0)
        if det and det.get('base_x_mm') is not None:
            detection = det
            break
        time.sleep(0.5)

    if not detection:
        logger.error("No detection from TRT server. Returning home.")
        arm.go_to_pose('home')
        return

    class_name = detection['class']
    conf = detection['confidence']
    bx = detection['base_x_mm'] / 1000.0
    by = detection['base_y_mm'] / 1000.0
    bz = detection['base_z_mm'] / 1000.0
    obj_height = detection.get('object_height_mm', 0)

    # Apply calibration offsets
    bx += calibration.get('detection_offset_x', 0.0)
    by += calibration.get('detection_offset_y', 0.0)
    bz += calibration.get('detection_offset_z', 0.0)

    # Apply per-class offsets
    cls_off = class_offsets.get(class_name, {})
    bx += cls_off.get('x', 0.0)
    by += cls_off.get('y', 0.0)
    bz += cls_off.get('z', 0.0)

    logger.info(f"Detected: {class_name} (conf={conf:.2f})")
    logger.info(f"  base_link: X={bx*1000:.0f} Y={by*1000:.0f} Z={bz*1000:.0f} mm (class offset: x={cls_off.get('x',0)*1000:.0f} z={cls_off.get('z',0)*1000:.0f})")

    bin_name = class_to_bin.get(class_name, 'landfill')
    logger.info(f"  Bin: {bin_name}")

    # Pickup sequence
    logger.info("--- PICKUP SEQUENCE ---")

    if not arm.go_to_position(bx, by, bz, label='pick'):
        logger.error("Failed to reach pick position. Aborting.")
        arm.go_to_pose('home')
        return

    arm.close_gripper(class_name=class_name)
    time.sleep(1.0)

    # Move to safe waypoint (clear of bins)
    safe_wp = arm.poses.get('safe_waypoint')
    if safe_wp:
        logger.info("Moving to safe waypoint...")
        arm.go_to_joints(safe_wp, label='safe_waypoint')

    bin_pose = f'pre_bin_{bin_name}'
    if bin_pose in arm.poses:
        logger.info(f"--- DEPOSITING in {bin_name} bin ---")
        arm.go_to_pose(bin_pose)
        arm.open_gripper()
        time.sleep(0.5)
    else:
        logger.error(f"Bin pose '{bin_pose}' not found. Dropping here.")
        arm.open_gripper()

    arm.go_to_pose('home')
    logger.info("=== PICKUP COMPLETE ===")


# ── Listen Mode (waits for /pickup_request from waste_tracker) ──

def switch_trt_pose(port, pose_name):
    """Switch TRT server's transform matrix."""
    import urllib.request
    try:
        req = urllib.request.Request(
            f'http://localhost:{port}/pose/{pose_name}',
            method='POST', data=b'')
        urllib.request.urlopen(req, timeout=2)
        return True
    except Exception:
        return False


def run_listen_mode(arm, config, port):
    """Listen for /pickup_request, execute pickup, signal /pickup_complete."""
    logger = arm.node.get_logger()
    logger.info("=== LISTEN MODE: waiting for /pickup_request ===")

    from std_msgs.msg import String
    calibration = config.get('calibration', {})
    class_to_bin = config.get('class_to_bin', {})
    class_offsets = config.get('class_offsets', {})
    search_pose = config.get('poses', {}).get('search', [0.0, 1.263, 1.354, -1.931])

    pickup_request = [None]  # mutable container for callback
    arm_pose_request = [None]  # arm pose request from waste_tracker
    intermediate_pose = config.get('poses', {}).get('intermediate', [0.0, 0.863, 1.331, -1.867])

    def request_cb(msg):
        pickup_request[0] = msg.data
        logger.info(f"Received pickup request: {msg.data}")

    def arm_pose_cb(msg):
        arm_pose_request[0] = msg.data
        logger.info(f"Received arm pose request: {msg.data}")

    sub = arm.node.create_subscription(String, '/pickup_request', request_cb, 10)
    arm_sub = arm.node.create_subscription(String, '/arm_pose_request', arm_pose_cb, 10)
    complete_pub = arm.node.create_publisher(String, '/pickup_complete', 10)
    arm_complete_pub = arm.node.create_publisher(String, '/arm_pose_complete', 10)

    logger.info("Listening on /pickup_request + /arm_pose_request... (Ctrl+C to stop)")

    while rclpy.ok():
        rclpy.spin_once(arm.node, timeout_sec=0.1)

        # Handle arm pose requests (from waste_tracker during approach)
        if arm_pose_request[0] is not None:
            pose_name = arm_pose_request[0]
            arm_pose_request[0] = None
            logger.info(f"=== ARM POSE: moving to {pose_name} ===")
            try:
                # Move arm to requested pose
                if pose_name == 'intermediate':
                    arm.go_to_joints(intermediate_pose, label='intermediate')
                elif pose_name == 'search':
                    arm.go_to_joints(search_pose, label='search')
                elif pose_name == 'home':
                    arm.go_to_pose('home')
                else:
                    logger.warn(f"Unknown arm pose: {pose_name}")

                # Switch TRT matrix to match
                switch_trt_pose(port, pose_name)
                time.sleep(0.3)  # brief settle

                logger.info(f"ARM POSE: {pose_name} reached")
                msg = String()
                msg.data = pose_name
                arm_complete_pub.publish(msg)
            except Exception as e:
                logger.error(f"ARM POSE error: {e}")
                msg = String()
                msg.data = f'failed:{pose_name}'
                arm_complete_pub.publish(msg)
            continue

        if pickup_request[0] is None:
            continue

        class_name = pickup_request[0]
        pickup_request[0] = None  # reset
        logger.info(f"=== PICKUP TRIGGERED: {class_name} ===")

        try:
            # Arm is already at intermediate pose (switched during approach)
            # TRT matrix is already set to intermediate
            # Just open gripper and detect

            # 1. Open gripper
            arm.open_gripper()

            # 2. Wait for camera to settle
            time.sleep(1.0)

            # 3. Fetch detection from TRT server
            logger.info("Detecting object from home pose...")
            detection = None
            for attempt in range(15):
                logger.info(f"  Attempt {attempt + 1}/15...")
                det = fetch_detection_from_trt(port, timeout=3.0)
                if det and det.get('base_x_mm') is not None:
                    detection = det
                    break
                time.sleep(0.3)

            if not detection:
                logger.error("No detection. Returning to search pose.")
                arm.go_to_joints(search_pose, label='search')
                switch_trt_pose(port, 'search')
                msg = String()
                msg.data = 'failed'
                complete_pub.publish(msg)
                continue

            # 6. Compute pick position
            bx = detection['base_x_mm'] / 1000.0
            by = detection['base_y_mm'] / 1000.0
            bz = detection['base_z_mm'] / 1000.0
            bx += calibration.get('detection_offset_x', 0.0)
            by += calibration.get('detection_offset_y', 0.0)
            bz += calibration.get('detection_offset_z', 0.0)

            det_class = detection.get('class', class_name)
            # Apply per-class offsets on top of global calibration
            cls_off = class_offsets.get(det_class, {})
            bx += cls_off.get('x', 0.0)
            by += cls_off.get('y', 0.0)
            bz += cls_off.get('z', 0.0)

            bin_name = class_to_bin.get(det_class, 'landfill')
            logger.info(f"Detected: {det_class} (conf={detection['confidence']:.2f})")
            logger.info(f"  base_link: X={bx*1000:.0f} Y={by*1000:.0f} Z={bz*1000:.0f} mm (class offset: x={cls_off.get('x',0)*1000:.0f} z={cls_off.get('z',0)*1000:.0f})")
            logger.info(f"  Bin: {bin_name}")

            # 7. Pick up
            logger.info("--- PICKUP SEQUENCE ---")
            if not arm.go_to_position(bx, by, bz, label='pick'):
                logger.error("Failed to reach pick position.")
                arm.go_to_joints(search_pose, label='search')
                switch_trt_pose(port, 'search')
                msg = String()
                msg.data = 'failed'
                complete_pub.publish(msg)
                continue

            arm.close_gripper(class_name=class_name)
            time.sleep(1.0)

            # 8a. Move to safe waypoint (clear of bins)
            safe_wp = arm.poses.get('safe_waypoint')
            if safe_wp:
                logger.info("Moving to safe waypoint...")
                arm.go_to_joints(safe_wp, label='safe_waypoint')

            # 8b. Sort to bin
            bin_pose = f'pre_bin_{bin_name}'
            if bin_pose in arm.poses:
                logger.info(f"--- DEPOSITING in {bin_name} bin ---")
                arm.go_to_pose(bin_pose)
                arm.open_gripper()
                time.sleep(0.5)
            else:
                logger.error(f"Bin pose '{bin_pose}' not found.")
                arm.open_gripper()

            # 9. Return to search pose
            logger.info("Returning to search pose...")
            arm.go_to_joints(search_pose, label='search')

            # 10. Switch TRT back to search matrix
            switch_trt_pose(port, 'search')

            # 11. Signal completion
            logger.info("=== PICKUP COMPLETE ===")
            msg = String()
            msg.data = 'success'
            complete_pub.publish(msg)

        except Exception as e:
            logger.error(f"Pickup error: {e}")
            import traceback
            traceback.print_exc()
            arm.go_to_joints(search_pose, label='search')
            switch_trt_pose(port, 'search')
            msg = String()
            msg.data = 'failed'
            complete_pub.publish(msg)


# ── Main ────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='RECLAIM Pick-and-Place')
    parser.add_argument('--test', action='store_true', help='Test mode: cycle poses')
    parser.add_argument('--camera', action='store_true', help='Camera mode: detect + pick + sort')
    parser.add_argument('--trt', action='store_true', help='TRT mode: fetch from TRT server (30 FPS)')
    parser.add_argument('--listen', action='store_true', help='Listen mode: wait for /pickup_request from waste_tracker')
    parser.add_argument('--bin', type=str, default=None,
                        help='Override bin (recyclable, compost, landfill)')
    parser.add_argument('--model', type=str, default='models/waste_yolo11n_v3_best.pt',
                        help='YOLO model path')
    parser.add_argument('--conf', type=float, default=0.35, help='YOLO confidence threshold')
    parser.add_argument('--port', type=int, default=8081, help='MJPEG/TRT server port')
    parser.add_argument('--config', type=str, default=None, help='Config YAML path')
    args = parser.parse_args()

    if args.config:
        config_path = args.config
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'pick_and_place.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    rclpy.init()
    node = rclpy.create_node('pick_and_place')
    logger = node.get_logger()

    try:
        arm = ArmController(node, config)

        if args.test:
            run_test_mode(arm, bin_name=args.bin)
        elif args.listen:
            run_listen_mode(arm, config, args.port)
        elif args.trt:
            run_trt_mode(arm, config, args.port)
        elif args.camera:
            start_mjpeg_server(args.port)
            run_camera_mode(arm, config, args.model, args.conf)
        else:
            logger.info("No mode specified. Use --test, --camera, or --trt")
    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if tf_node:
            tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
