#!/usr/bin/env python3
"""
waste_tracker.py — Autonomous scan-approach-pick state machine for RECLAIM.

This is the top-level autonomy node. It replaces Nav2 with a simpler
camera-driven approach:

  1. SCAN:     Rotate 360° in place, run YOLO on every frame, collect all
               detections with their 3D positions (via OAK-D stereo depth).
               After a full rotation, pick the nearest target.

  2. APPROACH: Drive toward the chosen target using visual servoing.
               Camera tracks the object — angular.z keeps it centered,
               linear.x drives forward. Slows down as it gets closer.

  3. ALIGN:    Stop driving. Fine-adjust heading so the target is centered
               and within the arm's reachable depth (150-450mm). Stabilize
               for a few frames to confirm position.

  4. PICK:     Stop motors. Execute the arm pickup sequence via serial
               commands through the PTY proxy (/tmp/teensy_arm). Arm does:
               home → prepick → pickup → grip → sort-to-bin → home.
               Then return to SCAN.

Data flow:
  ┌────────────────────────────────────────────────────┐
  │              waste_tracker node                     │
  │                                                    │
  │  OAK-D camera (internal, DepthAI v3 pipeline)     │
  │       ↓                                            │
  │  YOLO inference → 3D position (stereo depth)       │
  │       ↓                                            │
  │  State machine logic                               │
  │       ↓                                            │
  │  /cmd_vel (Twist) ──→ teensy_bridge ──→ motors     │
  │                                                    │
  │  /odom (Odometry) ←── teensy_bridge ←── encoders   │
  │                                                    │
  │  Arm commands ──→ /tmp/teensy_arm PTY ──→ Teensy   │
  │  (TeensyArmClient)                                 │
  │                                                    │
  │  /robot_state (String) ──→ for monitoring/debug    │
  └────────────────────────────────────────────────────┘

Prerequisites:
  - teensy_bridge must be running FIRST (owns /dev/ttyACM0, creates PTY)
  - OAK-D Lite on USB 3.0 port
  - YOLO model file on the MIC-711
  - Arm must be ARMed before autonomous pickup will work

Usage:
  # Terminal 1: start teensy_bridge (serial owner)
  ros2 launch reclaim_control teensy_bridge.launch.py

  # Terminal 2: start waste_tracker
  ros2 launch reclaim_bringup waste_tracker.launch.py

  # Or directly:
  ros2 run reclaim_bringup waste_tracker --ros-args \
      -p model_path:=models/waste_yolo11n_v3_best.engine \
      -p confidence:=0.35

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
# Add teensy_client to path so we can reuse TeensyArmClient for arm commands.
# This talks to /tmp/teensy_arm (PTY created by teensy_bridge), NOT the real
# serial port.  teensy_bridge forwards commands to the Teensy transparently.
# ---------------------------------------------------------------------------
_WS_SRC = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                        'src', 'reclaim_control', 'scripts')
if os.path.isdir(_WS_SRC):
    sys.path.insert(0, os.path.abspath(_WS_SRC))

try:
    from teensy_client import TeensyArmClient
except ImportError:
    TeensyArmClient = None


# ══════════════════════════════════════════════════════════════════════
#  Constants & helpers (reused from guided_pickup.py)
# ══════════════════════════════════════════════════════════════════════

CLASS_NAMES = {
    0: 'plastic_bottle', 1: 'aluminum_can', 2: 'cardboard',
    3: 'plastic_container', 4: 'cup', 5: 'chip_bag',
    6: 'styrofoam_container', 7: 'napkin', 8: 'paper_bag',
    9: 'apple', 10: 'orange',
}

BIN_MAP = {
    'plastic_bottle': 'recyclable', 'aluminum_can': 'recyclable',
    'cardboard': 'recyclable', 'plastic_container': 'recyclable',
    'cup': 'landfill', 'chip_bag': 'landfill',
    'styrofoam_container': 'landfill', 'napkin': 'compost',
    'paper_bag': 'compost', 'apple': 'compost', 'orange': 'compost',
}

BIN_NUMBER = {'recyclable': 1, 'compost': 2, 'landfill': 3}

HOLD = -999  # firmware sentinel: keep current joint angle


class State(Enum):
    IDLE = auto()
    SCAN = auto()
    TURN_TO_TARGET = auto()
    APPROACH = auto()
    ALIGN = auto()
    PICK = auto()


@dataclass
class Detection:
    """A single detected waste item with 3D position and pickup plan."""
    class_name: str
    confidence: float
    # Camera-frame position (mm)
    cam_x: float
    cam_y: float
    cam_z: float
    # Robot base-frame position (mm): x=forward, y=left, z=up
    robot_x: float
    robot_y: float
    robot_z: float
    # Pixel center in image
    cx_px: int
    cy_px: int
    # Bounding box size (for grip strategy)
    bbox_w: float
    bbox_h: float
    # ByteTrack persistent ID (None if tracker not available)
    track_id: Optional[int] = None

    @property
    def distance_mm(self) -> float:
        """Horizontal distance from robot to object (mm)."""
        return math.sqrt(self.robot_x ** 2 + self.robot_y ** 2)

    @property
    def bearing_rad(self) -> float:
        """Angle from robot forward axis to object (rad). + = left."""
        return math.atan2(self.robot_y, self.robot_x)


# ── Camera-to-robot coordinate transform ──────────────────────────────

def camera_to_robot(x_cam_mm, y_cam_mm, z_cam_mm,
                    tilt_deg: float = 15.0,
                    tx_mm: float = 0.0,
                    ty_mm: float = 0.0,
                    tz_mm: float = -150.0) -> Tuple[float, float, float]:
    """Transform camera optical frame → robot base frame.

    Camera optical frame: Z = forward, X = right, Y = down.
    Robot base frame:     X = forward, Y = left,  Z = up.

    The camera is mounted with a downward tilt and offset from base_link.

    Args:
        x_cam_mm, y_cam_mm, z_cam_mm: position in camera optical frame (mm)
        tilt_deg: camera downward tilt angle (degrees)
        tx/ty/tz_mm: camera position offset from base_link origin (mm)

    Returns:
        (x_robot, y_robot, z_robot) in robot base frame (mm)
    """
    # Step 1: camera optical → camera link (axis swap)
    x_link = float(z_cam_mm)      # depth → forward
    y_link = -float(x_cam_mm)     # right → left
    z_link = -float(y_cam_mm)     # down  → up

    # Step 2: undo camera tilt (rotation about Y axis)
    theta = math.radians(tilt_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    x_rot = x_link * cos_t + z_link * sin_t
    y_rot = y_link
    z_rot = -x_link * sin_t + z_link * cos_t

    # Step 3: translate by camera mounting offset
    return (x_rot + tx_mm, y_rot + ty_mm, z_rot + tz_mm)


# ── Depth sampling from stereo depth frame ────────────────────────────

def get_depth_for_bbox(depth_frame, x1, y1, x2, y2,
                       intrinsics=None) -> Tuple[float, float, float]:
    """Extract median depth and 3D camera-frame position for a bbox.

    Uses the center 50% of the bounding box to avoid edge noise.
    Returns (x_mm, y_mm, z_mm) in camera optical frame, or (0,0,0) on failure.
    """
    h, w = depth_frame.shape[:2]
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(w, int(x2)), min(h, int(y2))
    if x2 <= x1 or y2 <= y1:
        return 0.0, 0.0, 0.0

    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    bw, bh = (x2 - x1) // 4, (y2 - y1) // 4
    roi = depth_frame[max(0, cy - bh):min(h, cy + bh),
                      max(0, cx - bw):min(w, cx + bw)]
    if roi.size == 0:
        return 0.0, 0.0, 0.0

    valid = roi[(roi > 100) & (roi < 10000)]  # 10cm – 10m
    if valid.size == 0:
        return 0.0, 0.0, 0.0

    z_mm = float(np.median(valid))

    if intrinsics is not None:
        fx, fy = float(intrinsics[0][0]), float(intrinsics[1][1])
        cx_cam, cy_cam = float(intrinsics[0][2]), float(intrinsics[1][2])
    else:
        fx, fy = 500.0, 500.0
        cx_cam, cy_cam = 320.0, 240.0

    x_mm = (cx - cx_cam) * z_mm / fx
    y_mm = (cy - cy_cam) * z_mm / fy
    return x_mm, y_mm, z_mm


# ── Grip strategy (from guided_pickup.py) ─────────────────────────────

def determine_grip(class_name: str, bbox_w: float, bbox_h: float
                   ) -> Tuple[int, int]:
    """Choose grip angle (J6) and wrist orientation (J5).

    Returns (grip_angle_deg, j5_angle_deg).
    """
    small = ['aluminum_can', 'apple', 'orange']
    large = ['cardboard', 'paper_bag', 'styrofoam_container']

    if class_name in small:
        grip = 35
    elif class_name in large:
        grip = 55
    else:
        grip = 45

    # Wrist orientation from aspect ratio
    if bbox_w > bbox_h * 1.3:
        j5 = -60   # horizontal
    elif bbox_h > bbox_w * 1.3:
        j5 = 26    # vertical
    else:
        j5 = -60   # default

    return grip, j5


def build_pickup_commands(j1: float, j5: int, grip_angle: int,
                          bin_n: int) -> List[str]:
    """Build serial command sequence for a pickup cycle.

    Overrides J1 (base rotation) and J5 (wrist orient) for the descent,
    then uses firmware-taught bin poses for sorting.
    """
    cmds = [
        f"SET {j1:.1f} 133 65 -15 {j5} 0 T 750",        # home
        f"SET {j1:.1f} 75 65 20 {j5} 0 T 700",           # prepick
        f"SET {j1:.1f} 65 65 20 {j5} 0 T 700",           # pickup (open)
        f"GRIP {grip_angle} T 500",                        # grip
        "__PAUSE_200__",
        f"SET {j1:.1f} 75 65 20 {j5} {HOLD} T 700",      # prepick_carry
        f"SET {j1:.1f} 133 65 -15 {j5} {HOLD} T 700",    # home_carry
        f"POSE bin{bin_n}_pre T 1000",                     # bin approach
        f"POSE bin{bin_n}_drop T 700",                     # drop
        f"POSE bin{bin_n}_pre T 700",                      # retract
        "POSE home T 1000",                                # return home
    ]
    return cmds


# ══════════════════════════════════════════════════════════════════════
#  WasteTracker ROS2 Node
# ══════════════════════════════════════════════════════════════════════

class WasteTracker(Node):
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

        # SCAN parameters
        self.declare_parameter('scan_angular_vel', 0.3)     # rad/s rotation speed
        self.declare_parameter('scan_full_rotation', 6.28)   # ~2*pi rad

        # APPROACH parameters
        self.declare_parameter('approach_linear_vel', 0.12)  # m/s max forward speed
        self.declare_parameter('approach_angular_gain', 0.003)  # px offset → rad/s
        self.declare_parameter('approach_linear_gain', 0.0003)  # depth mm → m/s
        self.declare_parameter('approach_stop_distance_mm', 400.0)  # stop when this close

        # ALIGN parameters
        self.declare_parameter('align_pixel_tolerance', 30)  # px from image center
        self.declare_parameter('align_angular_vel', 0.15)    # rad/s for fine adjustment
        self.declare_parameter('align_stable_frames', 5)     # frames target must be stable
        self.declare_parameter('align_min_depth_mm', 150.0)  # arm reach min
        self.declare_parameter('align_max_depth_mm', 450.0)  # arm reach max

        # Obstacle avoidance (depth-based)
        self.declare_parameter('obstacle_min_depth_mm', 300.0)  # stop if anything closer
        self.declare_parameter('drive_only', False)  # stop at ALIGN, skip PICK (drivetrain testing)

        # ── Read parameters ───────────────────────────────────────────
        self.model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        self.pty_path = self.get_parameter('pty_path').value

        self.cam_tilt = self.get_parameter('camera_tilt_deg').value
        self.cam_tx = self.get_parameter('camera_tx_mm').value
        self.cam_ty = self.get_parameter('camera_ty_mm').value
        self.cam_tz = self.get_parameter('camera_tz_mm').value

        self.scan_vel = self.get_parameter('scan_angular_vel').value
        self.scan_target = self.get_parameter('scan_full_rotation').value

        self.approach_max_v = self.get_parameter('approach_linear_vel').value
        self.approach_w_gain = self.get_parameter('approach_angular_gain').value
        self.approach_v_gain = self.get_parameter('approach_linear_gain').value
        self.approach_stop_dist = self.get_parameter('approach_stop_distance_mm').value

        self.align_px_tol = self.get_parameter('align_pixel_tolerance').value
        self.align_w = self.get_parameter('align_angular_vel').value
        self.align_stable_needed = self.get_parameter('align_stable_frames').value
        self.align_min_depth = self.get_parameter('align_min_depth_mm').value
        self.align_max_depth = self.get_parameter('align_max_depth_mm').value

        self.obstacle_min = self.get_parameter('obstacle_min_depth_mm').value
        self.drive_only = self.get_parameter('drive_only').value

        # ── State machine ─────────────────────────────────────────────
        self.state = State.IDLE
        self.scan_start_yaw = 0.0
        self.scan_accumulated = 0.0
        self.scan_detections: List[Detection] = []
        self.current_target: Optional[Detection] = None
        self.align_stable_count = 0
        self.current_yaw = 0.0
        self.prev_yaw = None
        self.items_collected = 0
        self.drive_only_done = False
        self.filtered_pixel_offset = 0.0  # low-pass filtered pixel offset
        self.filtered_angular_z = 0.0     # low-pass filtered angular command
        self.last_angular_z = 0.0         # for angular ramp limiter
        self.target_lost_count = 0  # frames since target last seen
        self.target_lost_threshold = 50  # rescan after this many missed frames (~2.5s at 20Hz)

        # Detection confirmation — require consistent detection before committing
        self.confirm_count = 0
        self.confirm_threshold = 3  # need 3 consecutive frames with same object
        self.confirm_candidate = None  # (class_name, cx_px, cy_px, cam_z)

        # ByteTrack target locking — persistent tracker ID
        self.tracked_id = None  # ByteTrack ID of the object we're pursuing

        # Smoothing constants
        self.angular_deadband = 0.02      # rad/s — below this, drive straight
        self.angular_ramp_rate = 0.3      # rad/s² — max angular acceleration (lower = smoother)
        self.angular_alpha = 0.18         # EMA alpha — heavier smoothing for cleaner motion

        # ── PID controller for angular visual servoing ─────────────────
        # Replaces pure P-control with full PID for smooth, accurate tracking
        self.pid_integral = 0.0           # accumulated error (pixel·seconds)
        self.pid_prev_error = 0.0         # previous error for derivative
        self.pid_kp = 0.0010             # proportional gain (rad/s per pixel)
        self.pid_ki = 0.00004            # integral gain — halved to reduce windup overcorrection
        self.pid_kd = 0.0004             # derivative gain — reduced to lower noise amplification
        self.pid_integral_max = 50.0     # anti-windup clamp (max accumulated error)
        self.pid_dt = 0.05               # control loop period (20Hz = 50ms)

        # ── Kalman filter for detection tracking ───────────────────────
        # State: [cx, cy, vx, vy] (pixel position + velocity)
        # Measurement: [cx, cy] (YOLO bbox center)
        # Predicts position when YOLO misses frames, smooths noise
        self._init_kalman()

        # ── Camera / YOLO (run in background thread) ──────────────────
        self.camera_lock = threading.Lock()
        self.latest_detections: List[Detection] = []
        self.obstacle_ahead = False
        self.camera_ready = False
        self.camera_frame_count = 0
        self.image_width = 640
        self.image_height = 480

        # ── Arm client (via PTY proxy) ────────────────────────────────
        self.arm: Optional[TeensyArmClient] = None

        # ── Debug image publisher (annotated YOLO frames) ────────────
        self.cv_bridge = CvBridge()
        self.debug_frame = None
        self.debug_frame_lock = threading.Lock()

        # ── ROS publishers / subscribers ──────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.image_pub = self.create_publisher(Image, '/perception/debug_image', 1)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # Rescan trigger — publish anything to /rescan to restart state machine
        self.create_subscription(String, '/rescan', self._rescan_cb, 10)

        # ── Main loop timer (20 Hz state machine tick) ────────────────
        self.create_timer(0.05, self._tick)

        # ── State publish timer (2 Hz) ────────────────────────────────
        self.create_timer(0.5, self._publish_state)

        # ── Start camera thread ───────────────────────────────────────
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()

        self.get_logger().info(
            f'WasteTracker initialized: model={self.model_path}, '
            f'conf={self.confidence}, pty={self.pty_path}'
        )
        self.get_logger().info('Waiting for camera to initialize...')

    # ==================================================================
    #  Odometry callback — track robot heading for SCAN rotation
    # ==================================================================

    def _rescan_cb(self, msg: String):
        """Restart state machine — triggered by publishing to /rescan."""
        self.get_logger().info('=== RESCAN triggered — restarting state machine ===')
        self._stop_motors()
        self.drive_only_done = False
        self.target_lost_count = 0
        self.current_target = None
        self.tracked_id = None
        self._pid_reset()
        self._kalman_reset()
        self._enter_scan()

    def _odom_cb(self, msg: Odometry):
        """Extract yaw and position from odometry."""
        q = msg.pose.pose.orientation
        # Yaw from quaternion (robot is 2D, only z rotation matters)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)
        # Position for scan position hold
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    # ==================================================================
    #  Camera + YOLO thread (runs independently at camera FPS)
    # ==================================================================

    def _camera_loop(self):
        """Background thread: capture frames, run YOLO, update detections.

        This runs at ~15-30 FPS depending on the model. The state machine
        reads self.latest_detections from the main timer callback.
        """
        import depthai as dai
        from ultralytics import YOLO

        is_engine = self.model_path.endswith(('.engine', '.trt'))

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        model = YOLO(self.model_path, task='detect')
        self.get_logger().info('YOLO model loaded.')
        use_tracker = True  # try ByteTrack; disables permanently on first failure

        with dai.Pipeline() as pipeline:
            # ── RGB camera ──
            cam = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A)
            rgb_out = cam.requestOutput(
                (self.image_width, self.image_height)).createOutputQueue()

            # ── Stereo depth (aligned to RGB) ──
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

            # Read factory intrinsics
            calib = pipeline.getDefaultDevice().readCalibration()
            intrinsics = np.array(calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A,
                self.image_width, self.image_height))
            self.get_logger().info(
                f'Intrinsics: fx={intrinsics[0][0]:.1f}, '
                f'fy={intrinsics[1][1]:.1f}, '
                f'cx={intrinsics[0][2]:.1f}, '
                f'cy={intrinsics[1][2]:.1f}')

            self.camera_ready = True
            self.get_logger().info('Camera ready. Entering IDLE state.')

            img_cx = self.image_width / 2.0

            while pipeline.isRunning() and rclpy.ok():
                rgb_data = rgb_out.get()
                frame = rgb_data.getCvFrame()

                depth_data = depth_out.tryGet()
                depth_frame = depth_data.getCvFrame() if depth_data else None

                # ── YOLO inference with optional ByteTrack ──
                infer_conf = 0.20
                if use_tracker:
                    try:
                        results = model.track(frame, conf=infer_conf, verbose=False,
                                               persist=True, tracker="bytetrack.yaml")
                    except Exception as e:
                        self.get_logger().warn(f'ByteTrack failed, disabling permanently: {e}')
                        use_tracker = False
                        results = model(frame, conf=infer_conf, verbose=False)
                else:
                    results = model(frame, conf=infer_conf, verbose=False)
                if is_engine:
                    results[0].names = CLASS_NAMES
                boxes = results[0].boxes

                detections: List[Detection] = []

                if len(boxes) > 0 and depth_frame is not None:
                    if depth_frame.shape[:2] != (self.image_height,
                                                  self.image_width):
                        depth_frame = cv2.resize(
                            depth_frame,
                            (self.image_width, self.image_height),
                            interpolation=cv2.INTER_NEAREST)

                    # Extract tracker IDs if available
                    track_ids = None
                    if boxes.id is not None:
                        track_ids = boxes.id.cpu().numpy().astype(int)

                    for i, box in enumerate(boxes):
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
                            self.cam_tilt, self.cam_tx,
                            self.cam_ty, self.cam_tz)

                        cx_px = int((x1 + x2) / 2)
                        cy_px = int((y1 + y2) / 2)
                        bbox_w = float(x2 - x1)
                        bbox_h = float(y2 - y1)
                        tid = int(track_ids[i]) if track_ids is not None else None

                        detections.append(Detection(
                            class_name=name, confidence=conf,
                            cam_x=cam_x, cam_y=cam_y, cam_z=cam_z,
                            robot_x=robot_x, robot_y=robot_y,
                            robot_z=robot_z,
                            cx_px=cx_px, cy_px=cy_px,
                            bbox_w=bbox_w, bbox_h=bbox_h,
                            track_id=tid,
                        ))

                # ── Obstacle check (center strip of depth image) ──
                obstacle = False
                if depth_frame is not None:
                    # Check a vertical strip in the center 20% of the image
                    strip_l = int(self.image_width * 0.4)
                    strip_r = int(self.image_width * 0.6)
                    center_strip = depth_frame[:, strip_l:strip_r]
                    valid_center = center_strip[center_strip > 0]
                    if valid_center.size > 0:
                        min_depth = float(np.percentile(valid_center, 5))
                        obstacle = min_depth < self.obstacle_min

                # ── Filter detections: only small floor-level objects ──
                # Relaxed depth during tracking (stereo depth noisy during rotation)
                img_area = self.image_width * self.image_height
                tracking = self.state in (
                    State.TURN_TO_TARGET, State.APPROACH, State.ALIGN)
                max_depth = 3000 if tracking else 2500
                max_bbox_frac = 0.40 if tracking else 0.25  # close-range objects look bigger
                # Floor filter: bottom portion of image only
                # SCAN: bottom 75% to catch distant objects; tracking: bottom 80% (even more relaxed)
                min_cy_frac = 0.20 if tracking else 0.25
                detections = [d for d in detections
                              if d.cam_z < max_depth
                              and d.cy_px > self.image_height * min_cy_frac
                              and (d.bbox_w * d.bbox_h) < img_area * max_bbox_frac
                              and (d.bbox_w * d.bbox_h) > img_area * 0.003]

                # ── Draw annotations on frame for debug viewer ──
                annotated = frame.copy()
                for det in detections:
                    x1 = int(det.cx_px - det.bbox_w / 2)
                    y1 = int(det.cy_px - det.bbox_h / 2)
                    x2 = int(det.cx_px + det.bbox_w / 2)
                    y2 = int(det.cy_px + det.bbox_h / 2)
                    # Highlight tracked target in cyan, others in green
                    is_locked = (det.track_id is not None and
                                 det.track_id == self.tracked_id)
                    color = (255, 255, 0) if is_locked else (0, 255, 0)
                    thickness = 3 if is_locked else 2
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
                    tid_str = f' T{det.track_id}' if det.track_id is not None else ''
                    label = f'{det.class_name} {det.confidence:.2f} {det.cam_z:.0f}mm{tid_str}'
                    cv2.putText(annotated, label, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                # State overlay
                cv2.putText(annotated, f'State: {self.state.name}', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                # Publish debug image every frame for smooth Foxglove feed
                try:
                    img_msg = self.cv_bridge.cv2_to_imgmsg(annotated, 'bgr8')
                    self.image_pub.publish(img_msg)
                except Exception:
                    pass

                # ── Update shared state (thread-safe) ──
                with self.camera_lock:
                    self.latest_detections = detections
                    self.obstacle_ahead = obstacle
                    self.camera_frame_count += 1

    # ==================================================================
    #  Main state machine tick (20 Hz)
    # ==================================================================

    def _tick(self):
        """Main control loop — runs at 20 Hz from ROS timer."""
        if not self.camera_ready:
            return

        # Auto-transition from IDLE to SCAN once camera is ready
        # But not if we already completed a drive_only cycle
        if self.state == State.IDLE and self.items_collected == 0 and not self.drive_only_done:
            self._enter_scan()
            return

        if self.state == State.SCAN:
            self._tick_scan()
        elif self.state == State.TURN_TO_TARGET:
            self._tick_turn_to_target()
        elif self.state == State.APPROACH:
            self._tick_approach()
        elif self.state == State.ALIGN:
            self._tick_align()
        elif self.state == State.PICK:
            pass  # pickup runs synchronously in a thread

    # ── SCAN state ────────────────────────────────────────────────────

    def _enter_scan(self):
        self.state = State.SCAN
        self.scan_start_yaw = self.current_yaw
        self.scan_accumulated = 0.0
        self.prev_yaw = self.current_yaw
        self.scan_detections.clear()
        self.confirm_count = 0
        self.confirm_candidate = None
        self.tracked_id = None  # reset tracker lock
        # Store starting position for position hold during spin
        self.scan_start_x = getattr(self, 'odom_x', 0.0)
        self.scan_start_y = getattr(self, 'odom_y', 0.0)
        self.get_logger().info('=== SCAN: starting 360° rotation ===')

    def _tick_scan(self):
        """Rotate in place, collect detections, stop after full rotation."""
        # Track accumulated rotation using yaw deltas (handles wrapping)
        if self.prev_yaw is not None:
            delta = self.current_yaw - self.prev_yaw
            # Normalize delta to [-pi, pi] for wrap-around
            delta = math.atan2(math.sin(delta), math.cos(delta))
            # Count rotation in commanded direction, but also accept small
            # negative drift. Only ignore tiny noise (< 0.005 rad ≈ 0.3°)
            if abs(delta) > 0.005:
                # Weight: full credit for positive (commanded), half for negative (drift)
                if delta > 0:
                    self.scan_accumulated += delta
                else:
                    self.scan_accumulated += abs(delta) * 0.3  # partial credit for drift
        self.prev_yaw = self.current_yaw

        # Collect detections from the camera thread
        with self.camera_lock:
            frame_dets = list(self.latest_detections)

        # Filter: only small floor-level objects within detection range
        # Bottom 75% of image — distant floor objects appear higher in frame
        # with 15° downward camera tilt
        img_area = self.image_width * self.image_height
        frame_dets = [d for d in frame_dets
                      if d.cam_z < 2500
                      and d.cy_px > self.image_height * 0.25   # bottom 75% (catches distant objects)
                      and (d.bbox_w * d.bbox_h) < img_area * 0.25
                      and (d.bbox_w * d.bbox_h) > img_area * 0.002]

        for det in frame_dets:
            # De-duplicate: skip if we already have a detection of the
            # same class within 200mm (same object seen from similar angle)
            is_dup = False
            for existing in self.scan_detections:
                if (existing.class_name == det.class_name and
                        abs(existing.cam_z - det.cam_z) < 200):
                    # Keep the higher-confidence one
                    if det.confidence > existing.confidence:
                        self.scan_detections.remove(existing)
                    else:
                        is_dup = True
                    break
            if not is_dup:
                self.scan_detections.append(det)

        # Early exit: if we have a confirmed detection nearby, go for it
        if frame_dets:
            # Filter: only objects within 2500mm AND in bottom 85% of image (floor level)
            nearby = [d for d in frame_dets
                      if d.cam_z < 2500 and d.cy_px > self.image_height * 0.15]
            if nearby:
                # Pick closest object (not highest confidence) — go for nearest target
                best = min(nearby, key=lambda d: d.cam_z)
                if best.confidence >= 0.35:
                    # Check if this matches our confirmation candidate
                    if (self.confirm_candidate is not None and
                            best.class_name == self.confirm_candidate[0] and
                            abs(best.cx_px - self.confirm_candidate[1]) < 80 and
                            abs(best.cam_z - self.confirm_candidate[3]) < 500):
                        self.confirm_count += 1
                        self.confirm_candidate = (best.class_name, best.cx_px,
                                                   best.cy_px, best.cam_z)
                    else:
                        # New candidate — reset counter
                        self.confirm_count = 1
                        self.confirm_candidate = (best.class_name, best.cx_px,
                                                   best.cy_px, best.cam_z)

                    if self.confirm_count >= self.confirm_threshold:
                        self._stop_motors()
                        self.current_target = best
                        self.tracked_id = best.track_id  # lock ByteTrack ID
                        self.confirm_count = 0
                        self.confirm_candidate = None
                        self.get_logger().info(
                            f'SCAN: confirmed lock on {best.class_name} '
                            f'(conf={best.confidence:.2f}, depth={best.cam_z:.0f}mm, '
                            f'trackID={best.track_id})')
                        self._enter_turn_to_target()
                        return
            else:
                # No nearby detections — reset confirmation
                self.confirm_count = 0
                self.confirm_candidate = None

        # Publish rotation command — slow down when we have a candidate
        twist = Twist()
        if self.confirm_count > 0:
            twist.angular.z = self.scan_vel * 0.4  # slow down to confirm
        else:
            twist.angular.z = self.scan_vel
        self.cmd_vel_pub.publish(twist)

        # Check if full rotation is complete
        if self.scan_accumulated >= self.scan_target:
            self._stop_motors()

            if not self.scan_detections:
                self.get_logger().info(
                    'SCAN complete — no targets found. Rescanning...')
                self._enter_scan()  # rescan
                return

            # Sort by distance, pick nearest
            self.scan_detections.sort(key=lambda d: d.distance_mm)
            self.current_target = self.scan_detections[0]

            self.get_logger().info(
                f'SCAN complete — {len(self.scan_detections)} targets found. '
                f'Nearest: {self.current_target.class_name} '
                f'@ {self.current_target.distance_mm:.0f}mm')

            self._enter_turn_to_target()

    # ── TURN_TO_TARGET state ──────────────────────────────────────────

    def _enter_turn_to_target(self):
        self.state = State.TURN_TO_TARGET
        self.filtered_pixel_offset = 0.0
        self.filtered_angular_z = 0.0
        self.last_angular_z = 0.0
        self.target_lost_count = 0
        self._pid_reset()
        self._kalman_reset()
        self._stop_motors()
        self.get_logger().info(
            f'=== TURN_TO_TARGET: rotating to face {self.current_target.class_name} ===')

    def _tick_turn_to_target(self):
        """Rotate in place until the target is centered in the image."""
        with self.camera_lock:
            frame_dets = list(self.latest_detections)

        target_det = self._find_target_in_detections(frame_dets)

        # During TURN_TO_TARGET, if exact match fails, accept ANY detection
        # The object shifts in frame during rotation so position matching fails
        if target_det is None and frame_dets:
            # Accept the closest detection by class, or just the highest confidence
            target_class = self.current_target.class_name if self.current_target else None
            same_class = [d for d in frame_dets if d.class_name == target_class]
            if same_class:
                target_det = same_class[0]
            else:
                # Accept highest confidence detection — we're just trying to center
                target_det = max(frame_dets, key=lambda d: d.confidence)

        if target_det is None:
            self.target_lost_count += 1
            if self.target_lost_count >= self.target_lost_threshold:
                self._stop_motors()
                self.get_logger().warn('TURN_TO_TARGET: target lost for too long — rescanning')
                self._enter_scan()
            else:
                # Coast on Kalman prediction — but limit to last known direction
                # Clamp prediction offset to prevent runaway turning
                pred_cx, _ = self._kalman_predict()
                img_cx = self.image_width / 2.0
                pred_offset = pred_cx - img_cx
                # Clamp prediction to ±100px — don't chase wild extrapolations
                pred_offset = max(-100, min(100, pred_offset))
                if abs(pred_offset) > 20:
                    angular_z = self._pid_angular(pred_offset)
                    angular_z = max(-0.08, min(0.08, angular_z))  # very slow while lost
                    angular_z = self._smooth_angular(angular_z)
                    twist = Twist()
                    twist.angular.z = angular_z
                    self.cmd_vel_pub.publish(twist)
                else:
                    self._stop_motors()  # centered prediction — just wait
            return

        self.target_lost_count = 0
        self.current_target = target_det

        # Kalman filter: update with measurement for smooth tracking
        filtered_cx, _ = self._kalman_update(float(target_det.cx_px),
                                              float(target_det.cy_px))
        img_cx = self.image_width / 2.0
        pixel_offset = filtered_cx - img_cx

        if abs(pixel_offset) > 20:
            # Only correct the portion outside the deadzone for smoother approach
            corrected_offset = pixel_offset - 20.0 * (1.0 if pixel_offset > 0 else -1.0)
            raw_angular_z = self._pid_angular(corrected_offset)
            raw_angular_z = max(-0.12, min(0.12, raw_angular_z))
            angular_z = self._smooth_angular(raw_angular_z)
            twist = Twist()
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
        else:
            # Centered — smooth stop then transition to APPROACH
            self._smooth_angular(0.0)  # let ramp wind down
            self._stop_motors()
            self.get_logger().info(
                f'TURN_TO_TARGET: centered (offset={pixel_offset:.0f}px) '
                f'→ approaching')
            self._enter_approach()

    # ── APPROACH state ────────────────────────────────────────────────

    def _enter_approach(self):
        self.state = State.APPROACH
        self.filtered_pixel_offset = 0.0
        self.filtered_angular_z = 0.0
        self.last_angular_z = 0.0
        self.target_lost_count = 0
        self._pid_reset()
        self._stop_motors()
        self.get_logger().info(
            f'=== APPROACH: driving toward {self.current_target.class_name} ===')

    def _tick_approach(self):
        """Drive toward target using visual servoing.

        Angular: filtered proportional control to keep target centered.
        Linear:  proportional to distance, ramping down as we get closer.
        """
        # Check for obstacles
        with self.camera_lock:
            obstacle = self.obstacle_ahead
            frame_dets = list(self.latest_detections)

        # Obstacle check disabled during approach — the target IS the close object
        # if obstacle:
        #     self._stop_motors()
        #     self.get_logger().warn('APPROACH: obstacle detected — stopping')
        #     return

        target_det = self._find_target_in_detections(frame_dets)

        if target_det is None:
            self.target_lost_count += 1
            if self.target_lost_count >= self.target_lost_threshold:
                self._stop_motors()
                # If we were already close, just stop — don't spin into the object
                last_depth = self.current_target.cam_z if self.current_target else 9999
                if last_depth < 500:
                    self.get_logger().info(
                        f'APPROACH: target lost at close range ({last_depth:.0f}mm) — stopping')
                    if self.drive_only:
                        self.drive_only_done = True
                    self.state = State.IDLE
                else:
                    self.get_logger().warn('APPROACH: target lost for too long — rescanning')
                    self._enter_scan()
            else:
                # Kalman prediction: coast forward but clamp prediction
                pred_cx, _ = self._kalman_predict()
                img_cx = self.image_width / 2.0
                pred_offset = max(-100, min(100, pred_cx - img_cx))
                angular_z = self._pid_angular(pred_offset)
                angular_z = max(-0.08, min(0.08, angular_z))  # gentle while lost
                angular_z = self._smooth_angular(angular_z)
                twist = Twist()
                twist.linear.x = 0.04
                twist.angular.z = angular_z
                self.cmd_vel_pub.publish(twist)
            return

        # Target found — reset lost counter
        self.target_lost_count = 0
        self.current_target = target_det

        # ── Visual servoing with Kalman + PID ──
        filtered_cx, _ = self._kalman_update(float(target_det.cx_px),
                                              float(target_det.cy_px))
        img_cx = self.image_width / 2.0
        pixel_offset = filtered_cx - img_cx

        # Pixel deadzone: if target is roughly centered, drive straight
        # This eliminates micro-corrections that cause jittery heading
        if abs(pixel_offset) < 15:
            angular_z = self._smooth_angular(0.0)  # smooth decay to zero
        else:
            # Only correct the portion outside the deadzone
            corrected_offset = pixel_offset - 15.0 * (1.0 if pixel_offset > 0 else -1.0)
            raw_angular_z = self._pid_angular(corrected_offset)
            # Tight clamp — gentle corrections while driving
            raw_angular_z = max(-0.10, min(0.10, raw_angular_z))
            angular_z = self._smooth_angular(raw_angular_z)

        # Linear: proportional to distance with smooth deceleration
        dist = target_det.cam_z  # mm
        if dist <= self.approach_stop_dist:
            self._stop_motors()
            self._enter_align()
            return

        # Speed ramps from max at >1500mm down to 0.04 at stop distance
        dist_ratio = (dist - self.approach_stop_dist) / 1100.0  # normalized 0-1
        dist_ratio = max(0.0, min(1.0, dist_ratio))
        linear_x = 0.04 + (self.approach_max_v - 0.04) * dist_ratio
        linear_x = max(0.04, min(self.approach_max_v, linear_x))

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    # ── ALIGN state ───────────────────────────────────────────────────

    def _enter_align(self):
        self.state = State.ALIGN
        self.align_stable_count = 0
        self.align_lost_count = 0
        self.get_logger().info('=== ALIGN: fine-positioning for pickup ===')

    def _tick_align(self):
        """Fine-adjust heading so target is centered and within reach."""
        with self.camera_lock:
            frame_dets = list(self.latest_detections)

        target_det = self._find_target_in_detections(frame_dets)

        # --- Fallback: accept ANY detection at close range ---
        if target_det is None and frame_dets:
            target_class = self.current_target.class_name if self.current_target else ''
            same_class = [d for d in frame_dets if d.class_name == target_class]
            if same_class:
                target_det = same_class[0]
            else:
                # At close range, just take whatever we see
                target_det = max(frame_dets, key=lambda d: d.confidence)

        if target_det is None:
            self.align_lost_count += 1
            # Patience: coast on Kalman prediction for 15 frames (~0.75s)
            if self.align_lost_count >= 15:
                self._stop_motors()
                last_depth = self.current_target.cam_z if self.current_target else 9999
                if last_depth < 500:
                    self.get_logger().info(
                        f'ALIGN: target lost at close range ({last_depth:.0f}mm) — stopping')
                    if self.drive_only:
                        self.drive_only_done = True
                    self.state = State.IDLE
                else:
                    self.get_logger().warn('ALIGN: target lost — rescanning')
                    self._enter_scan()
            # Hold position while lost (don't move)
            return

        # Target found — reset lost counter
        self.align_lost_count = 0
        self.current_target = target_det

        # Kalman-filtered position for smooth centering
        filtered_cx, _ = self._kalman_update(float(target_det.cx_px),
                                              float(target_det.cy_px))
        img_cx = self.image_width / 2.0
        pixel_offset = filtered_cx - img_cx

        # Check if centered — use PID control for smooth, precise centering
        if abs(pixel_offset) > self.align_px_tol:
            # PID rotation (same controller, shared integral for continuity)
            angular_z = self._pid_angular(pixel_offset)
            # Clamp to align speed limit
            angular_z = max(-self.align_w, min(self.align_w, angular_z))
            angular_z = self._smooth_angular(angular_z)
            twist = Twist()
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            self.align_stable_count = 0
            return

        # Centered — stop motors
        self._stop_motors()

        # Check depth is within arm reach
        depth = target_det.cam_z
        if depth > self.align_max_depth:
            # Too far — creep forward
            twist = Twist()
            twist.linear.x = 0.04
            self.cmd_vel_pub.publish(twist)
            self.align_stable_count = 0
            return
        # If too close, just stop — don't reverse (causes jittery backward movement)

        # Target is centered AND in reach — count stable frames
        self.align_stable_count += 1
        if self.align_stable_count >= self.align_stable_needed:
            self.get_logger().info(
                f'ALIGN: locked on {target_det.class_name} '
                f'@ {depth:.0f}mm, offset={pixel_offset:.0f}px')
            if self.drive_only:
                self.get_logger().info(
                    'ALIGN: drive_only mode — stopping here (PICK disabled)')
                self._stop_motors()
                self.drive_only_done = True
                self.state = State.IDLE
            else:
                self._enter_pick()

    # ── PICK state ────────────────────────────────────────────────────

    def _enter_pick(self):
        self.state = State.PICK
        self._stop_motors()
        self.get_logger().info(
            f'=== PICK: picking up {self.current_target.class_name} ===')

        # Run pickup in a thread so we don't block the ROS executor
        pick_thread = threading.Thread(target=self._execute_pick, daemon=True)
        pick_thread.start()

    def _execute_pick(self):
        """Execute the full arm pickup sequence (blocking).

        Connects to arm via PTY, sends pickup commands, waits for completion.
        """
        target = self.current_target
        if target is None:
            self.get_logger().error('PICK: no target — returning to SCAN')
            self._enter_scan()
            return

        # ── Determine pickup parameters ──
        bin_name = BIN_MAP.get(target.class_name, 'landfill')
        bin_n = BIN_NUMBER.get(bin_name, 1)
        grip_angle, j5_angle = determine_grip(
            target.class_name, target.bbox_w, target.bbox_h)

        # Compute J1 rotation to face the object
        # robot_x = forward, robot_y = left → bearing angle
        base_j1 = -32.0  # default home J1 (from guided_pickup.py)
        if abs(target.robot_x) > 1.0:
            j1_delta = math.degrees(math.atan2(-target.robot_y, target.robot_x))
        else:
            j1_delta = 0.0
        j1_adj = max(-148.0, min(148.0, base_j1 + j1_delta))

        self.get_logger().info(
            f'PICK plan: {target.class_name} → bin {bin_n} ({bin_name}), '
            f'J1={j1_adj:.1f}, grip={grip_angle}, J5={j5_angle}')

        # ── Connect to arm via PTY ──
        if TeensyArmClient is None:
            self.get_logger().error('PICK: TeensyArmClient not available')
            self._enter_scan()
            return

        try:
            arm = TeensyArmClient(self.pty_path, baud=115200, boot_wait_s=0.5)
            arm.open()
        except Exception as e:
            self.get_logger().error(f'PICK: failed to connect to arm: {e}')
            self._enter_scan()
            return

        try:
            # Build and execute command sequence
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
                    arm.command("POSE home T 1000", wait_for_done=True,
                                timeout_s=10.0)
                    success = False
                    break

            if success:
                self.items_collected += 1
                self.get_logger().info(
                    f'PICK: SUCCESS — {target.class_name} → {bin_name}. '
                    f'Total collected: {self.items_collected}')
            else:
                self.get_logger().warn('PICK: FAILED — returning to scan')

        finally:
            arm.close()

        # Return to scanning
        self._enter_scan()

    # ==================================================================
    #  Helper methods
    # ==================================================================

    # ── Kalman filter ─────────────────────────────────────────────────

    def _init_kalman(self):
        """Initialize a 4-state Kalman filter for tracking detection position.

        State vector: [cx, cy, vx, vy]  (pixel position + velocity)
        Measurement:  [cx, cy]           (YOLO bbox center)

        The Kalman filter does two things:
        1. PREDICT: estimates where the object is even when YOLO misses a frame
        2. UPDATE: corrects the estimate when a measurement is available

        Math:
            Prediction:  x̂(k|k-1) = F·x̂(k-1|k-1)       (state transition)
                         P(k|k-1) = F·P(k-1|k-1)·Fᵀ + Q  (covariance prediction)

            Update:      K = P(k|k-1)·Hᵀ·(H·P(k|k-1)·Hᵀ + R)⁻¹  (Kalman gain)
                         x̂(k|k) = x̂(k|k-1) + K·(z - H·x̂(k|k-1))  (state update)
                         P(k|k) = (I - K·H)·P(k|k-1)               (cov update)

        Where:
            F = state transition (constant velocity model)
            H = measurement matrix (we only observe position, not velocity)
            Q = process noise (how much we expect the object to move unexpectedly)
            R = measurement noise (how noisy YOLO bbox centers are)
        """
        self.kf = cv2.KalmanFilter(4, 2)  # 4 states, 2 measurements

        dt = self.pid_dt  # 50ms between frames

        # State transition matrix F: constant velocity model
        # [cx]     [1 0 dt 0] [cx]
        # [cy]  =  [0 1 0 dt] [cy]
        # [vx]     [0 0 1  0] [vx]
        # [vy]     [0 0 0  1] [vy]
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=np.float32)

        # Measurement matrix H: we only observe [cx, cy]
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)

        # Process noise Q: how much the object moves unexpectedly between frames
        # Higher = trust measurements more, lower = trust model more
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 3.0

        # Measurement noise R: how noisy YOLO bbox centers are (~10-15px jitter)
        # Higher = smooth more (trust model), lower = react faster (trust YOLO)
        # Increased to 25 for much smoother tracking — less reactive to single-frame jumps
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 25.0

        # Initial covariance P: start uncertain
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 100.0

        # Initial state
        self.kf.statePost = np.array([320, 240, 0, 0], dtype=np.float32)

        self.kalman_initialized = False

    def _kalman_predict(self) -> Tuple[float, float]:
        """Predict the next object position (no measurement available).

        Returns predicted (cx, cy) in pixels.
        """
        prediction = self.kf.predict()
        return float(prediction[0]), float(prediction[1])

    def _kalman_update(self, cx: float, cy: float) -> Tuple[float, float]:
        """Update Kalman filter with a new YOLO measurement.

        Returns filtered (cx, cy) in pixels — much smoother than raw YOLO.
        """
        if not self.kalman_initialized:
            # First measurement — initialize state to this position
            self.kf.statePost = np.array([cx, cy, 0, 0], dtype=np.float32)
            self.kalman_initialized = True
            return cx, cy

        self.kf.predict()
        measurement = np.array([cx, cy], dtype=np.float32)
        corrected = self.kf.correct(measurement)
        return float(corrected[0]), float(corrected[1])

    def _kalman_reset(self):
        """Reset Kalman filter when switching targets."""
        self.kalman_initialized = False
        self.kf.statePost = np.array([320, 240, 0, 0], dtype=np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 100.0

    # ── PID controller ─────────────────────────────────────────────────

    def _pid_angular(self, pixel_error: float) -> float:
        """PID controller for angular visual servoing.

        Math:
            u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de/dt

        Where:
            e(t) = pixel offset from image center (pixels)
            u(t) = angular velocity command (rad/s)

        Kp (proportional): Corrects current error. Higher = faster response
            but more overshoot.

        Ki (integral): Accumulates past errors. Eliminates steady-state offset
            (robot settling with object slightly off-center). Uses anti-windup
            clamping to prevent integral from growing unbounded.

        Kd (derivative): Reacts to rate of change of error. Dampens oscillation
            — when error is decreasing fast, Kd reduces the correction to prevent
            overshoot. This is what stops the left-right swaying.

        Anti-windup: Clamps integral to [-pid_integral_max, +pid_integral_max]
            to prevent windup when the target is far off-center for a long time.
        """
        # Proportional term
        p_term = self.pid_kp * pixel_error

        # Integral term with anti-windup
        self.pid_integral += pixel_error * self.pid_dt
        self.pid_integral = max(-self.pid_integral_max,
                                min(self.pid_integral_max, self.pid_integral))
        i_term = self.pid_ki * self.pid_integral

        # Derivative term (rate of change of error)
        d_term = self.pid_kd * (pixel_error - self.pid_prev_error) / self.pid_dt
        self.pid_prev_error = pixel_error

        # Combined PID output (negative because positive pixel offset = turn right)
        angular_z = -(p_term + i_term + d_term)

        # Clamp to safe angular velocity
        angular_z = max(-0.3, min(0.3, angular_z))

        return angular_z

    def _pid_reset(self):
        """Reset PID state when switching targets or states."""
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0

    def _smooth_angular(self, raw_angular_z: float, dt: float = 0.05) -> float:
        """Apply EMA, smooth taper, and sinusoidal ramp to angular command.

        Three-stage smoothing:
        1. EMA low-pass filter on PID output (removes high-freq jitter)
        2. Smooth taper near zero (replaces hard deadband — no more bang-bang)
        3. Sinusoidal ramp limiter (S-curve acceleration for smooth starts/stops)
        """
        # Stage 1: EMA low-pass filter (alpha=0.25 = heavy smoothing)
        raw_angular_z = self.angular_alpha * raw_angular_z + (1.0 - self.angular_alpha) * self.filtered_angular_z
        self.filtered_angular_z = raw_angular_z

        # Stage 2: Smooth taper — quadratic fade near zero instead of hard cutoff
        # This eliminates the bang-bang jerk from the old deadband
        if abs(raw_angular_z) < self.angular_deadband:
            # Scale quadratically: at deadband edge = full, at zero = zero
            scale = (raw_angular_z / self.angular_deadband) ** 2
            raw_angular_z = raw_angular_z * scale

        # Stage 3: Sinusoidal ramp limiter — S-curve instead of linear ramp
        # Smoother acceleration profile (no jerk at start/end of ramp)
        max_delta = self.angular_ramp_rate * dt
        delta = raw_angular_z - self.last_angular_z
        if abs(delta) > max_delta:
            # Sinusoidal blend: ease in/out based on how far through the ramp we are
            t = max_delta / abs(delta)  # 0..1 progress
            smooth_t = 0.5 * (1.0 - math.cos(math.pi * t))  # S-curve
            raw_angular_z = self.last_angular_z + delta * smooth_t

        self.last_angular_z = raw_angular_z
        return raw_angular_z

    def _find_target_in_detections(self, detections: List[Detection]
                                    ) -> Optional[Detection]:
        """Find the tracked target in the latest detection list.

        Priority order:
        1. ByteTrack ID match (persistent across frames — best)
        2. Position-based match (within 80px of last known)
        3. Class name match
        4. Any detection within radius (last resort)
        """
        if not detections or self.current_target is None:
            return None

        # Primary: ByteTrack ID match (most reliable)
        if self.tracked_id is not None:
            id_matches = [d for d in detections if d.track_id == self.tracked_id]
            if id_matches:
                return id_matches[0]

        ref_cx = self.current_target.cx_px
        ref_cy = self.current_target.cy_px

        # Secondary: match by position (within 80px of last known location)
        nearby = [d for d in detections
                  if abs(d.cx_px - ref_cx) < 80 and abs(d.cy_px - ref_cy) < 80]

        if nearby:
            nearby.sort(key=lambda d:
                        (d.cx_px - ref_cx) ** 2 + (d.cy_px - ref_cy) ** 2)
            match = nearby[0]
            # Adopt this detection's track ID for future matching
            if match.track_id is not None:
                self.tracked_id = match.track_id
            return match

        # Tertiary: match by class name
        target_class = self.current_target.class_name
        candidates = [d for d in detections if d.class_name == target_class]

        if candidates:
            candidates.sort(key=lambda d:
                            (d.cx_px - ref_cx) ** 2 + (d.cy_px - ref_cy) ** 2)
            match = candidates[0]
            if match.track_id is not None:
                self.tracked_id = match.track_id
            return match

        # Last resort: any detection (wider radius at close range / ALIGN)
        if detections:
            detections_sorted = sorted(detections, key=lambda d:
                            (d.cx_px - ref_cx) ** 2 + (d.cy_px - ref_cy) ** 2)
            max_radius = 250 if self.state == State.ALIGN else 150
            if ((detections_sorted[0].cx_px - ref_cx) ** 2 +
                (detections_sorted[0].cy_px - ref_cy) ** 2) < max_radius ** 2:
                match = detections_sorted[0]
                if match.track_id is not None:
                    self.tracked_id = match.track_id
                return match

        return None

    def _stop_motors(self):
        """Publish zero velocity."""
        self.cmd_vel_pub.publish(Twist())

    def _publish_state(self):
        """Publish current state for monitoring."""
        msg = String()
        info = f'{self.state.name}'
        if self.state == State.SCAN:
            progress = min(100, int(
                self.scan_accumulated / self.scan_target * 100))
            info += f' ({progress}%, {len(self.scan_detections)} found)'
        elif self.state in (State.TURN_TO_TARGET, State.APPROACH, State.ALIGN) and self.current_target:
            info += (f' → {self.current_target.class_name} '
                     f'@ {self.current_target.cam_z:.0f}mm')
        elif self.state == State.PICK and self.current_target:
            info += f' {self.current_target.class_name}'
        info += f' | collected: {self.items_collected}'
        msg.data = info
        self.state_pub.publish(msg)

    def destroy_node(self):
        self._stop_motors()
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = WasteTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
