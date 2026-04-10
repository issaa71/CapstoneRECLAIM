#!/usr/bin/env python3
"""
waste_tracker_v2.py — Stop-Look-Drive approach for RECLAIM.

KEY DIFFERENCE from v1 (waste_tracker.py):
  v1 uses continuous visual servoing — camera feeds PID controller in real-time,
  which causes jittery corrections, motion blur, and castor scrub.

  v2 uses STOP-LOOK-DRIVE — robot stops, takes a clean static frame, decides
  what to do, executes a short blind maneuver, then stops and looks again.
  No Kalman filter, no PID, no EMA smoothing needed.

States:
  1. SCAN:     PIVOT TURN (one wheel stopped, other spins) instead of in-place
               rotation. Castors at back trace a natural arc — minimal scrub.

  2. TURN:     STOP → take frame → compute pixel offset → execute timed turn
               (one wheel only = pivot) → STOP → re-check. Repeat until centered.

  3. DRIVE:    STOP → take frame → confirm target centered → drive BLIND with
               heading hold for a short burst (0.3-0.5s) → STOP → re-check.
               No visual servoing during motion — heading hold keeps it straight.

  4. ALIGN:    STOP → take frame → small correction if needed → repeat until
               stable for N frames within arm reach.

  5. PICK:     Same as v1.

Authors: Shady Siam, Issa Ahmed
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ---------------------------------------------------------------------------
# Reuse shared code from v1
# ---------------------------------------------------------------------------
_WS_SRC = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                        'src', 'reclaim_control', 'scripts')
if os.path.isdir(_WS_SRC):
    sys.path.insert(0, os.path.abspath(_WS_SRC))

try:
    from teensy_client import TeensyArmClient
except ImportError:
    TeensyArmClient = None

# Import shared utilities from v1
from reclaim_bringup.waste_tracker import (
    CLASS_NAMES, BIN_MAP, BIN_NUMBER, HOLD,
    Detection, camera_to_robot, get_depth_for_bbox,
    determine_grip, build_pickup_commands,
)


class State(Enum):
    IDLE = auto()
    SCAN = auto()
    TURN = auto()      # stop-look-turn (replaces TURN_TO_TARGET)
    DRIVE = auto()     # stop-look-drive (replaces APPROACH)
    ALIGN = auto()
    PICK = auto()


class WasteTrackerV2(Node):
    def __init__(self):
        super().__init__('waste_tracker')

        # ── ROS Parameters ────────────────────────────────────────────
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('pty_path', '/tmp/teensy_arm')

        # Camera transform parameters
        self.declare_parameter('camera_tilt_deg', 15.0)
        self.declare_parameter('camera_tx_mm', 0.0)
        self.declare_parameter('camera_ty_mm', 0.0)
        self.declare_parameter('camera_tz_mm', -150.0)

        # SCAN parameters — pivot turn (one wheel stopped, other spins)
        self.declare_parameter('scan_angular_vel', 0.25)     # rad/s rotation
        self.declare_parameter('scan_full_rotation', 6.28)
        self.declare_parameter('wheel_separation', 0.470)    # for pivot turn calc

        # DRIVE parameters
        self.declare_parameter('approach_linear_vel', 0.10)  # m/s
        self.declare_parameter('approach_stop_distance_mm', 400.0)
        self.declare_parameter('drive_burst_time', 0.5)      # seconds of blind driving per burst

        # ALIGN parameters
        self.declare_parameter('align_pixel_tolerance', 35)
        self.declare_parameter('align_stable_frames', 5)
        self.declare_parameter('align_min_depth_mm', 150.0)
        self.declare_parameter('align_max_depth_mm', 450.0)

        self.declare_parameter('drive_only', False)

        # ── Read parameters ───────────────────────────────────────────
        self.model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        self.pty_path = self.get_parameter('pty_path').value

        self.cam_tilt = self.get_parameter('camera_tilt_deg').value
        self.cam_tx = self.get_parameter('camera_tx_mm').value
        self.cam_ty = self.get_parameter('camera_ty_mm').value
        self.cam_tz = self.get_parameter('camera_tz_mm').value

        self.scan_w = self.get_parameter('scan_angular_vel').value
        self.scan_target = self.get_parameter('scan_full_rotation').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        # Pivot turn: to stop one wheel, set linear.x = w * L / 2
        # This makes v_left = v - w*L/2 = 0 (left wheel stationary)
        # Robot pivots around the left wheel
        self.scan_pivot_v = self.scan_w * self.wheel_sep / 2.0

        self.approach_max_v = self.get_parameter('approach_linear_vel').value
        self.approach_stop_dist = self.get_parameter('approach_stop_distance_mm').value
        self.drive_burst = self.get_parameter('drive_burst_time').value

        self.align_px_tol = self.get_parameter('align_pixel_tolerance').value
        self.align_stable_needed = self.get_parameter('align_stable_frames').value
        self.align_min_depth = self.get_parameter('align_min_depth_mm').value
        self.align_max_depth = self.get_parameter('align_max_depth_mm').value

        self.drive_only = self.get_parameter('drive_only').value

        # ── State machine ─────────────────────────────────────────────
        self.state = State.IDLE
        self.scan_accumulated = 0.0
        self.prev_yaw = None
        self.current_yaw = 0.0
        self.scan_detections: List[Detection] = []
        self.current_target: Optional[Detection] = None
        self.items_collected = 0
        self.drive_only_done = False

        # SCAN confirmation
        self.confirm_count = 0
        self.confirm_threshold = 3
        self.confirm_candidate = None

        # TURN state
        self.turn_attempts = 0
        self.turn_max_attempts = 20  # give up after this many turn-check cycles

        # DRIVE state
        self.drive_burst_active = False
        self.drive_burst_end = 0.0
        self.drive_heading_target = 0.0
        self.drive_lost_count = 0

        # ALIGN state
        self.align_stable_count = 0
        self.align_lost_count = 0

        # ── Camera / YOLO (run in background thread) ──────────────────
        self.camera_lock = threading.Lock()
        self.latest_detections: List[Detection] = []
        self.camera_ready = False
        self.camera_frame_count = 0
        self.image_width = 640
        self.image_height = 480

        # ── Debug image publisher ─────────────────────────────────────
        self.cv_bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/perception/debug_image', 1)

        # ── ROS publishers / subscribers ──────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(String, '/rescan', self._rescan_cb, 10)

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(0.05, self._tick)       # 20 Hz state machine
        self.create_timer(0.5, self._publish_state)  # 2 Hz status

        # ── Start camera thread ───────────────────────────────────────
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()

        self.get_logger().info(
            f'WasteTrackerV2 (stop-look-drive): model={self.model_path}')

    # ==================================================================
    #  Callbacks
    # ==================================================================

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def _rescan_cb(self, msg: String):
        self.get_logger().info('=== RESCAN triggered ===')
        self._stop_motors()
        self.drive_only_done = False
        self.current_target = None
        self._enter_scan()

    # ==================================================================
    #  Camera + YOLO thread (same as v1)
    # ==================================================================

    def _camera_loop(self):
        import depthai as dai
        from ultralytics import YOLO

        is_engine = self.model_path.endswith(('.engine', '.trt'))
        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        model = YOLO(self.model_path, task='detect')
        self.get_logger().info('YOLO model loaded.')

        with dai.Pipeline() as pipeline:
            cam = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A)
            rgb_out = cam.requestOutput(
                (self.image_width, self.image_height)).createOutputQueue()

            stereo = pipeline.create(dai.node.StereoDepth).build(
                autoCreateCameras=True)
            stereo.setDefaultProfilePreset(
                dai.node.StereoDepth.PresetMode.DEFAULT)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(self.image_width, self.image_height)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)
            stereo.setExtendedDisparity(True)
            depth_out = stereo.depth.createOutputQueue()

            pipeline.start()
            self.get_logger().info('OAK-D pipeline started.')

            calib = pipeline.getDefaultDevice().readCalibration()
            intrinsics = np.array(calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A,
                self.image_width, self.image_height))
            self.get_logger().info(
                f'Intrinsics: fx={intrinsics[0][0]:.1f}, fy={intrinsics[1][1]:.1f}')

            self.camera_ready = True
            self.get_logger().info('Camera ready.')

            while pipeline.isRunning() and rclpy.ok():
                rgb_data = rgb_out.get()
                frame = rgb_data.getCvFrame()

                depth_data = depth_out.tryGet()
                depth_frame = depth_data.getCvFrame() if depth_data else None

                infer_conf = 0.20
                results = model(frame, conf=infer_conf, verbose=False)
                if is_engine:
                    results[0].names = CLASS_NAMES
                boxes = results[0].boxes

                detections: List[Detection] = []

                if len(boxes) > 0 and depth_frame is not None:
                    if depth_frame.shape[:2] != (self.image_height, self.image_width):
                        depth_frame = cv2.resize(
                            depth_frame, (self.image_width, self.image_height),
                            interpolation=cv2.INTER_NEAREST)

                    for box in boxes:
                        cls_id = int(box.cls.cpu().numpy()[0])
                        conf = float(box.conf.cpu().numpy()[0])
                        name = CLASS_NAMES.get(cls_id, f'class{cls_id}')
                        x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]

                        cam_x, cam_y, cam_z = get_depth_for_bbox(
                            depth_frame, x1, y1, x2, y2, intrinsics)
                        if cam_z < 1:
                            continue

                        robot_x, robot_y, robot_z = camera_to_robot(
                            cam_x, cam_y, cam_z,
                            self.cam_tilt, self.cam_tx, self.cam_ty, self.cam_tz)

                        cx_px = int((x1 + x2) / 2)
                        cy_px = int((y1 + y2) / 2)
                        bbox_w = float(x2 - x1)
                        bbox_h = float(y2 - y1)

                        detections.append(Detection(
                            class_name=name, confidence=conf,
                            cam_x=cam_x, cam_y=cam_y, cam_z=cam_z,
                            robot_x=robot_x, robot_y=robot_y, robot_z=robot_z,
                            cx_px=cx_px, cy_px=cy_px,
                            bbox_w=bbox_w, bbox_h=bbox_h,
                        ))

                # Floor filter
                img_area = self.image_width * self.image_height
                tracking = self.state in (State.TURN, State.DRIVE, State.ALIGN)
                max_depth = 3000 if tracking else 2500
                max_bbox_frac = 0.40 if tracking else 0.25
                min_cy_frac = 0.20 if tracking else 0.25
                detections = [d for d in detections
                              if d.cam_z < max_depth
                              and d.cy_px > self.image_height * min_cy_frac
                              and (d.bbox_w * d.bbox_h) < img_area * max_bbox_frac
                              and (d.bbox_w * d.bbox_h) > img_area * 0.003]

                # Debug annotations
                annotated = frame.copy()
                for det in detections:
                    x1 = int(det.cx_px - det.bbox_w / 2)
                    y1 = int(det.cy_px - det.bbox_h / 2)
                    x2 = int(det.cx_px + det.bbox_w / 2)
                    y2 = int(det.cy_px + det.bbox_h / 2)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f'{det.class_name} {det.confidence:.2f} {det.cam_z:.0f}mm'
                    cv2.putText(annotated, label, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(annotated, f'V2: {self.state.name}', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                try:
                    img_msg = self.cv_bridge.cv2_to_imgmsg(annotated, 'bgr8')
                    self.image_pub.publish(img_msg)
                except Exception:
                    pass

                with self.camera_lock:
                    self.latest_detections = detections
                    self.camera_frame_count += 1

    # ==================================================================
    #  State machine tick (20 Hz)
    # ==================================================================

    def _tick(self):
        if not self.camera_ready:
            return

        if self.state == State.IDLE and self.items_collected == 0 and not self.drive_only_done:
            self._enter_scan()
            return

        if self.state == State.SCAN:
            self._tick_scan()
        elif self.state == State.TURN:
            self._tick_turn()
        elif self.state == State.DRIVE:
            self._tick_drive()
        elif self.state == State.ALIGN:
            self._tick_align()

    # ── SCAN: arc turn ────────────────────────────────────────────────

    def _enter_scan(self):
        self.state = State.SCAN
        self.scan_accumulated = 0.0
        self.prev_yaw = self.current_yaw
        self.scan_detections.clear()
        self.confirm_count = 0
        self.confirm_candidate = None
        self.get_logger().info(
            f'=== SCAN: pivot-turn 360° (w={self.scan_w}, '
            f'v={self.scan_pivot_v:.3f}) ===')

    def _tick_scan(self):
        # Track rotation
        if self.prev_yaw is not None:
            delta = self.current_yaw - self.prev_yaw
            delta = math.atan2(math.sin(delta), math.cos(delta))
            if abs(delta) > 0.005:
                if delta > 0:
                    self.scan_accumulated += delta
                else:
                    self.scan_accumulated += abs(delta) * 0.3
        self.prev_yaw = self.current_yaw

        # Collect detections
        with self.camera_lock:
            frame_dets = list(self.latest_detections)

        img_area = self.image_width * self.image_height
        frame_dets = [d for d in frame_dets
                      if d.cam_z < 2500
                      and d.cy_px > self.image_height * 0.25
                      and (d.bbox_w * d.bbox_h) < img_area * 0.25
                      and (d.bbox_w * d.bbox_h) > img_area * 0.002]

        for det in frame_dets:
            is_dup = False
            for existing in self.scan_detections:
                if (existing.class_name == det.class_name and
                        abs(existing.cam_z - det.cam_z) < 200):
                    if det.confidence > existing.confidence:
                        self.scan_detections.remove(existing)
                    else:
                        is_dup = True
                    break
            if not is_dup:
                self.scan_detections.append(det)

        # Early exit with confirmation
        if frame_dets:
            nearby = [d for d in frame_dets
                      if d.cam_z < 2500 and d.cy_px > self.image_height * 0.15]
            if nearby:
                best = min(nearby, key=lambda d: d.cam_z)
                if best.confidence >= 0.35:
                    if (self.confirm_candidate is not None and
                            best.class_name == self.confirm_candidate[0] and
                            abs(best.cx_px - self.confirm_candidate[1]) < 80 and
                            abs(best.cam_z - self.confirm_candidate[3]) < 500):
                        self.confirm_count += 1
                        self.confirm_candidate = (best.class_name, best.cx_px,
                                                   best.cy_px, best.cam_z)
                    else:
                        self.confirm_count = 1
                        self.confirm_candidate = (best.class_name, best.cx_px,
                                                   best.cy_px, best.cam_z)
                    if self.confirm_count >= self.confirm_threshold:
                        self._stop_motors()
                        self.current_target = best
                        self.confirm_count = 0
                        self.get_logger().info(
                            f'SCAN: locked {best.class_name} '
                            f'(conf={best.confidence:.2f}, depth={best.cam_z:.0f}mm)')
                        self._enter_turn()
                        return
            else:
                self.confirm_count = 0
                self.confirm_candidate = None

        # Pivot turn: linear.x = w * L/2 makes left wheel stationary
        # Robot pivots around left wheel — castors trace natural arc
        twist = Twist()
        if self.confirm_count > 0:
            w = self.scan_w * 0.4
            twist.angular.z = w
            twist.linear.x = w * self.wheel_sep / 2.0
        else:
            twist.angular.z = self.scan_w
            twist.linear.x = self.scan_pivot_v
        self.cmd_vel_pub.publish(twist)

        # Check full rotation
        if self.scan_accumulated >= self.scan_target:
            self._stop_motors()
            if not self.scan_detections:
                self.get_logger().info('SCAN complete — no targets. Rescanning...')
                self._enter_scan()
                return
            self.scan_detections.sort(key=lambda d: d.distance_mm)
            self.current_target = self.scan_detections[0]
            self.get_logger().info(
                f'SCAN complete — {len(self.scan_detections)} targets. '
                f'Nearest: {self.current_target.class_name} '
                f'@ {self.current_target.distance_mm:.0f}mm')
            self._enter_turn()

    # ── TURN: stop-look-turn ──────────────────────────────────────────

    def _enter_turn(self):
        self.state = State.TURN
        self.turn_attempts = 0
        self._stop_motors()
        self.get_logger().info(
            f'=== TURN: centering {self.current_target.class_name} ===')

    def _tick_turn(self):
        """Stop-look-turn cycle: stop, read frame, compute offset, turn a bit."""
        # Get current detections (robot is stopped — clean frame)
        with self.camera_lock:
            frame_dets = list(self.latest_detections)

        target_det = self._find_target(frame_dets)

        if target_det is None:
            self.turn_attempts += 1
            if self.turn_attempts >= self.turn_max_attempts:
                self.get_logger().warn('TURN: target lost — rescanning')
                self._enter_scan()
            # Stay stopped, wait for next frame
            return

        self.current_target = target_det
        self.turn_attempts = 0

        img_cx = self.image_width / 2.0
        pixel_offset = target_det.cx_px - img_cx

        if abs(pixel_offset) <= 30:
            # Centered — move to DRIVE
            self.get_logger().info(
                f'TURN: centered (offset={pixel_offset:.0f}px) → DRIVE')
            self._enter_drive()
            return

        # Compute turn duration based on pixel offset
        # Rough mapping: 320px offset ≈ 45° ≈ 0.785 rad
        # At 0.15 rad/s, that's ~5.2s for full half-frame offset
        angle_estimate = abs(pixel_offset) / self.image_width * 0.8  # rough rad
        turn_speed = 0.15  # rad/s — slow for accuracy
        turn_duration = angle_estimate / turn_speed
        turn_duration = max(0.08, min(1.5, turn_duration))  # clamp

        # Direction: positive pixel offset = object is right = turn right (negative angular.z)
        direction = -1.0 if pixel_offset > 0 else 1.0

        # Execute timed turn (blocking but short)
        twist = Twist()
        twist.angular.z = direction * turn_speed
        self.cmd_vel_pub.publish(twist)
        time.sleep(turn_duration)
        self._stop_motors()

        # Small settling delay for clean next frame
        time.sleep(0.1)

        self.get_logger().info(
            f'TURN: offset={pixel_offset:.0f}px → turned {direction*turn_duration:.2f}s')

    # ── DRIVE: stop-look-drive ────────────────────────────────────────

    def _enter_drive(self):
        self.state = State.DRIVE
        self.drive_burst_active = False
        self.drive_lost_count = 0
        self._stop_motors()
        self.get_logger().info(
            f'=== DRIVE: approaching {self.current_target.class_name} ===')

    def _tick_drive(self):
        """Stop-look-drive cycle: check target, drive blind burst, re-check."""
        # During burst — just wait for it to finish
        if self.drive_burst_active:
            if time.monotonic() >= self.drive_burst_end:
                # Burst done — stop and re-check
                self._stop_motors()
                self.drive_burst_active = False
                time.sleep(0.15)  # settle for clean frame
            return

        # Not driving — check target position
        with self.camera_lock:
            frame_dets = list(self.latest_detections)

        target_det = self._find_target(frame_dets)

        if target_det is None:
            self.drive_lost_count += 1
            if self.drive_lost_count >= 30:  # ~1.5s at 20Hz
                last_depth = self.current_target.cam_z if self.current_target else 9999
                if last_depth < 500:
                    self.get_logger().info(
                        f'DRIVE: target lost at close range ({last_depth:.0f}mm) — stopping')
                    if self.drive_only:
                        self.drive_only_done = True
                    self.state = State.IDLE
                else:
                    self.get_logger().warn('DRIVE: target lost — rescanning')
                    self._enter_scan()
            return

        self.drive_lost_count = 0
        self.current_target = target_det

        # Check if close enough for ALIGN
        dist = target_det.cam_z
        if dist <= self.approach_stop_dist:
            self._stop_motors()
            self._enter_align()
            return

        # Check if target is still centered — if not, go back to TURN
        img_cx = self.image_width / 2.0
        pixel_offset = target_det.cx_px - img_cx
        if abs(pixel_offset) > 80:
            self.get_logger().info(
                f'DRIVE: target off-center ({pixel_offset:.0f}px) → TURN')
            self._enter_turn()
            return

        # Target centered and far enough — start a drive burst
        # Speed proportional to distance
        dist_ratio = (dist - self.approach_stop_dist) / 1100.0
        dist_ratio = max(0.0, min(1.0, dist_ratio))
        linear_x = 0.05 + (self.approach_max_v - 0.05) * dist_ratio

        # Shorter bursts when closer
        burst_time = self.drive_burst * dist_ratio + 0.15
        burst_time = max(0.15, min(self.drive_burst, burst_time))

        # Lock heading for straight driving (bridge heading hold does the rest)
        self.drive_heading_target = self.current_yaw

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = 0.0  # heading hold in bridge handles corrections
        self.cmd_vel_pub.publish(twist)
        self.drive_burst_active = True
        self.drive_burst_end = time.monotonic() + burst_time

        self.get_logger().info(
            f'DRIVE: burst {burst_time:.2f}s @ {linear_x:.2f}m/s, '
            f'depth={dist:.0f}mm, offset={pixel_offset:.0f}px')

    # ── ALIGN: stop-check-adjust ──────────────────────────────────────

    def _enter_align(self):
        self.state = State.ALIGN
        self.align_stable_count = 0
        self.align_lost_count = 0
        self._stop_motors()
        self.get_logger().info('=== ALIGN: fine-positioning ===')

    def _tick_align(self):
        with self.camera_lock:
            frame_dets = list(self.latest_detections)

        target_det = self._find_target(frame_dets)

        # Fallback: accept any detection at close range
        if target_det is None and frame_dets:
            target_class = self.current_target.class_name if self.current_target else ''
            same_class = [d for d in frame_dets if d.class_name == target_class]
            if same_class:
                target_det = same_class[0]
            else:
                target_det = max(frame_dets, key=lambda d: d.confidence)

        if target_det is None:
            self.align_lost_count += 1
            if self.align_lost_count >= 15:
                self._stop_motors()
                last_depth = self.current_target.cam_z if self.current_target else 9999
                if last_depth < 500:
                    self.get_logger().info(
                        f'ALIGN: lost at close range ({last_depth:.0f}mm) — done')
                    if self.drive_only:
                        self.drive_only_done = True
                    self.state = State.IDLE
                else:
                    self.get_logger().warn('ALIGN: target lost — rescanning')
                    self._enter_scan()
            return

        self.align_lost_count = 0
        self.current_target = target_det

        img_cx = self.image_width / 2.0
        pixel_offset = target_det.cx_px - img_cx

        # If off-center, do a small timed correction (not continuous servoing)
        if abs(pixel_offset) > self.align_px_tol:
            # Small timed turn
            turn_speed = 0.10
            turn_time = abs(pixel_offset) / self.image_width * 0.5
            turn_time = max(0.05, min(0.4, turn_time))
            direction = -1.0 if pixel_offset > 0 else 1.0

            twist = Twist()
            twist.angular.z = direction * turn_speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(turn_time)
            self._stop_motors()
            time.sleep(0.1)
            self.align_stable_count = 0
            return

        # Centered — stop
        self._stop_motors()

        # Check depth
        depth = target_det.cam_z
        if depth > self.align_max_depth:
            # Creep forward
            twist = Twist()
            twist.linear.x = 0.04
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.2)
            self._stop_motors()
            time.sleep(0.1)
            self.align_stable_count = 0
            return

        # Target centered AND in reach
        self.align_stable_count += 1
        if self.align_stable_count >= self.align_stable_needed:
            self.get_logger().info(
                f'ALIGN: locked on {target_det.class_name} '
                f'@ {depth:.0f}mm, offset={pixel_offset:.0f}px')
            if self.drive_only:
                self.get_logger().info('ALIGN: drive_only — done')
                self._stop_motors()
                self.drive_only_done = True
                self.state = State.IDLE
            else:
                self._enter_pick()

    # ── PICK ──────────────────────────────────────────────────────────

    def _enter_pick(self):
        self.state = State.PICK
        self._stop_motors()
        self.get_logger().info(
            f'=== PICK: {self.current_target.class_name} ===')
        pick_thread = threading.Thread(target=self._execute_pick, daemon=True)
        pick_thread.start()

    def _execute_pick(self):
        """Same pickup logic as v1."""
        target = self.current_target
        if target is None:
            self._enter_scan()
            return

        bin_name = BIN_MAP.get(target.class_name, 'landfill')
        bin_n = BIN_NUMBER.get(bin_name, 1)
        grip_angle, j5_angle = determine_grip(target.class_name, target.bbox_w, target.bbox_h)

        base_j1 = -32.0
        if abs(target.robot_x) > 1.0:
            j1_delta = math.degrees(math.atan2(-target.robot_y, target.robot_x))
        else:
            j1_delta = 0.0
        j1_adj = max(-148.0, min(148.0, base_j1 + j1_delta))

        self.get_logger().info(
            f'PICK plan: {target.class_name} → bin {bin_n} ({bin_name})')

        if TeensyArmClient is None:
            self.get_logger().error('PICK: TeensyArmClient not available')
            self._enter_scan()
            return

        try:
            arm = TeensyArmClient(self.pty_path, baud=115200, boot_wait_s=0.5)
            arm.open()
        except Exception as e:
            self.get_logger().error(f'PICK: failed to connect: {e}')
            self._enter_scan()
            return

        try:
            commands = build_pickup_commands(j1_adj, j5_angle, grip_angle, bin_n)
            success = True
            for cmd in commands:
                if cmd == "__PAUSE_200__":
                    time.sleep(0.2)
                    continue
                result = arm.command(cmd, wait_for_done=True,
                                     timeout_s=12.0, echo=True)
                if result.timed_out:
                    self.get_logger().error(f'PICK: timeout on {cmd}')
                    arm.command("STOP", wait_for_done=False, timeout_s=2.0)
                    arm.command("POSE home T 1000", wait_for_done=True, timeout_s=10.0)
                    success = False
                    break
            if success:
                self.items_collected += 1
                self.get_logger().info(
                    f'PICK: SUCCESS — {target.class_name} → {bin_name}. '
                    f'Total: {self.items_collected}')
        finally:
            arm.close()

        self._enter_scan()

    # ==================================================================
    #  Helpers
    # ==================================================================

    def _find_target(self, detections: List[Detection]) -> Optional[Detection]:
        """Find tracked target — simpler than v1, position + class matching."""
        if not detections or self.current_target is None:
            return None

        ref_cx = self.current_target.cx_px
        ref_cy = self.current_target.cy_px

        # Position match (within 100px — wider since we stop between checks)
        nearby = [d for d in detections
                  if abs(d.cx_px - ref_cx) < 100 and abs(d.cy_px - ref_cy) < 100]
        if nearby:
            nearby.sort(key=lambda d:
                        (d.cx_px - ref_cx) ** 2 + (d.cy_px - ref_cy) ** 2)
            return nearby[0]

        # Class match
        target_class = self.current_target.class_name
        candidates = [d for d in detections if d.class_name == target_class]
        if candidates:
            candidates.sort(key=lambda d:
                            (d.cx_px - ref_cx) ** 2 + (d.cy_px - ref_cy) ** 2)
            return candidates[0]

        # Any detection within 200px
        all_sorted = sorted(detections, key=lambda d:
                            (d.cx_px - ref_cx) ** 2 + (d.cy_px - ref_cy) ** 2)
        if all_sorted and ((all_sorted[0].cx_px - ref_cx) ** 2 +
                           (all_sorted[0].cy_px - ref_cy) ** 2) < 200 ** 2:
            return all_sorted[0]

        return None

    def _stop_motors(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_state(self):
        msg = String()
        info = f'V2:{self.state.name}'
        if self.state == State.SCAN:
            progress = min(100, int(self.scan_accumulated / self.scan_target * 100))
            info += f' ({progress}%, {len(self.scan_detections)} found)'
        elif self.current_target:
            info += f' → {self.current_target.class_name} @ {self.current_target.cam_z:.0f}mm'
        info += f' | collected: {self.items_collected}'
        msg.data = info
        self.state_pub.publish(msg)

    def destroy_node(self):
        self._stop_motors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WasteTrackerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
