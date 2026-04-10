#!/usr/bin/env python3
"""
verify_camera_transform.py — Verify camera-to-arm TF with live MJPEG stream

Uses OAK-D Lite + YOLO + stereo depth with burst median (5-frame),
transforms detections from camera_optical_frame to base_link via tf2.
Overlays base_link XYZ on the video stream + pickup zone ROI.

Requires: move_group.launch.py running (publishes the static TFs).

Usage:
    python3 verify_camera_transform.py
    python3 verify_camera_transform.py --port 8081 --burst 5
    python3 verify_camera_transform.py --no-camera  # manual TF test

View in browser: http://127.0.0.1:8081
SSH tunnel: ssh -f -N -L 8081:127.0.0.1:8081 mic
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from geometry_msgs.msg import PointStamped

# Pickup zone ROI (pixel coordinates, tuned for home/look_down pose)
ROI_X_MIN = 10
ROI_X_MAX = 500
ROI_Y_MIN = 90
ROI_Y_MAX = 460

# Depth thresholds (mm)
DEPTH_MIN_MM = 150
DEPTH_MAX_MM = 600

# Global frame buffer for MJPEG stream
g_jpeg_frame = None
g_jpeg_lock = threading.Lock()


# ── MJPEG Web Server ────────────────────────────────────────────

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with g_jpeg_lock:
                        frame = g_jpeg_frame
                    if frame is not None:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress HTTP logs


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def start_mjpeg_server(port):
    server = ThreadedHTTPServer(('0.0.0.0', port), MJPEGHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


# ── TF2 Node ────────────────────────────────────────────────────

class TransformVerifier(Node):
    def __init__(self):
        super().__init__('verify_camera_transform')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def wait_for_tf(self, timeout=15.0):
        self.get_logger().info('Waiting for TF: base_link → camera_optical_frame ...')
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            try:
                self.tf_buffer.lookup_transform(
                    'base_link', 'camera_optical_frame',
                    rclpy.time.Time(), timeout=Duration(seconds=0.5))
                self.get_logger().info('TF available!')
                return True
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                pass
        self.get_logger().error(
            'TF not available after %.0fs. Is move_group.launch.py running?' % timeout)
        return False

    def transform_point(self, x_cam, y_cam, z_cam):
        pt = PointStamped()
        pt.header.frame_id = 'camera_optical_frame'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(x_cam)
        pt.point.y = float(y_cam)
        pt.point.z = float(z_cam)
        try:
            base_pt = self.tf_buffer.transform(
                pt, 'base_link', timeout=Duration(seconds=1.0))
            return base_pt.point.x, base_pt.point.y, base_pt.point.z
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            return None

    def print_transform_info(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
            tr = t.transform.translation
            rot = t.transform.rotation
            self.get_logger().info(
                f'base_link → camera_link: '
                f'XYZ=({tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f})  '
                f'QUAT=({rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f})')
        except Exception:
            pass
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'camera_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
            tr = t.transform.translation
            self.get_logger().info(
                f'base_link → camera_optical_frame: '
                f'XYZ=({tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f})')
        except Exception:
            pass


# ── Manual Mode ─────────────────────────────────────────────────

def run_manual_mode(node):
    print('\n=== Manual TF Test Mode ===')
    print('Enter camera_optical_frame coordinates (mm).')
    print('Format: x_mm y_mm z_mm   (Z=depth, X=right, Y=down)')
    print('Type "q" to quit.\n')
    while True:
        try:
            line = input('camera XYZ (mm): ').strip()
        except EOFError:
            break
        if line.lower() in ('q', 'quit', 'exit'):
            break
        parts = line.split()
        if len(parts) != 3:
            print('  Expected 3 numbers: x_mm y_mm z_mm')
            continue
        try:
            x_mm, y_mm, z_mm = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print('  Invalid numbers')
            continue
        result = node.transform_point(x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)
        if result:
            bx, by, bz = result
            print(f'  camera_optical: ({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm')
            print(f'  base_link:      ({bx*1000:.0f}, {by*1000:.0f}, {bz*1000:.0f}) mm')
            horiz = math.sqrt(bx**2 + by**2) * 1000
            print(f'  horiz dist from J1: {horiz:.0f} mm\n')


# ── Camera Mode ─────────────────────────────────────────────────

def run_camera_mode(node, model_path, conf_threshold, port, burst_count, settle_delay):
    global g_jpeg_frame

    import depthai as dai
    import cv2
    import numpy as np

    try:
        from ultralytics import YOLO
    except ImportError:
        node.get_logger().error('ultralytics not installed. Use --no-camera.')
        return

    CLASS_NAMES = {
        0: 'plastic_bottle', 1: 'aluminum_can', 2: 'cardboard',
        3: 'plastic_container', 4: 'cup', 5: 'chip_bag',
        6: 'styrofoam_container', 7: 'napkin', 8: 'paper_bag',
        9: 'apple', 10: 'orange',
    }
    is_engine = model_path.endswith('.engine') or model_path.endswith('.trt')

    # Start MJPEG server
    start_mjpeg_server(port)
    node.get_logger().info(f'MJPEG stream on http://0.0.0.0:{port}')

    node.get_logger().info(f'Loading model: {model_path}')
    model = YOLO(model_path, task='detect')
    node.get_logger().info('Model loaded.')

    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        rgb_out = cam.requestOutput((640, 480)).createOutputQueue()

        stereo = pipeline.create(dai.node.StereoDepth).build(autoCreateCameras=True)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(640, 480)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setExtendedDisparity(True)
        depth_out = stereo.depth.createOutputQueue()

        pipeline.start()
        node.get_logger().info('Camera + stereo depth started.')

        # Settle delay for camera startup
        time.sleep(settle_delay)

        calib = pipeline.getDefaultDevice().readCalibration()
        intrinsics = np.array(calib.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A, 640, 480))
        fx, fy = intrinsics[0][0], intrinsics[1][1]
        cx_cam, cy_cam = intrinsics[0][2], intrinsics[1][2]
        node.get_logger().info(
            f'Intrinsics: fx={fx:.1f} fy={fy:.1f} cx={cx_cam:.1f} cy={cy_cam:.1f}')
        node.get_logger().info(
            f'Burst: {burst_count} frames, Depth range: {DEPTH_MIN_MM}-{DEPTH_MAX_MM}mm')

        print('\n=== Camera Transform Verification ===')
        print(f'Stream: http://0.0.0.0:{port}')
        print('Place objects at known positions relative to J1.')
        print('Compare "base_link" output with tape-measure ground truth.')
        print('Press Ctrl+C to stop.\n')

        # Burst buffer for depth median
        depth_burst = []
        last_print = 0
        detection_info = []  # Store for overlay

        while pipeline.isRunning():
            rgb_data = rgb_out.get()
            frame = rgb_data.getCvFrame()

            depth_data = depth_out.tryGet()
            depth_frame = depth_data.getCvFrame() if depth_data else None

            # Collect burst frames
            if depth_frame is not None:
                depth_burst.append(depth_frame.copy())
                if len(depth_burst) > burst_count:
                    depth_burst.pop(0)

            # Run YOLO
            results = model(frame, conf=conf_threshold, verbose=False)
            if is_engine:
                results[0].names = CLASS_NAMES

            boxes = results[0].boxes
            now = time.time()
            display = frame.copy()

            # Draw pickup zone ROI
            cv2.rectangle(display, (ROI_X_MIN, ROI_Y_MIN), (ROI_X_MAX, ROI_Y_MAX),
                         (0, 255, 255), 2)
            cv2.putText(display, 'PICKUP ZONE', (ROI_X_MIN + 5, ROI_Y_MIN + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Process detections
            new_info = []
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = CLASS_NAMES.get(cls_id, f'class_{cls_id}')

                cx_box = int((x1 + x2) / 2)
                cy_box = int((y1 + y2) / 2)

                # Check if detection is in pickup zone
                in_roi = (ROI_X_MIN <= cx_box <= ROI_X_MAX and
                         ROI_Y_MIN <= cy_box <= ROI_Y_MAX)

                # Draw bbox
                color = (0, 255, 0) if in_roi else (0, 0, 255)
                cv2.rectangle(display, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(display, f'{name} {conf:.2f}',
                           (int(x1), int(y1) - 8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

                # Draw crosshair at center
                cv2.drawMarker(display, (cx_box, cy_box), color,
                              cv2.MARKER_CROSS, 15, 2)

                # Compute 3D position with burst median
                if len(depth_burst) >= burst_count and in_roi:
                    # BBox scale factor 0.5 (inner 50%)
                    bw = int((x2 - x1) * 0.25)
                    bh = int((y2 - y1) * 0.25)

                    z_samples = []
                    for df in depth_burst[-burst_count:]:
                        if df.shape[:2] != (480, 640):
                            df = cv2.resize(df, (640, 480),
                                          interpolation=cv2.INTER_NEAREST)
                        roi = df[max(0, cy_box - bh):min(480, cy_box + bh),
                                 max(0, cx_box - bw):min(640, cx_box + bw)]
                        valid = roi[(roi > DEPTH_MIN_MM) & (roi < DEPTH_MAX_MM)]
                        if valid.size > 0:
                            z_samples.append(float(np.median(valid)))

                    if len(z_samples) >= burst_count // 2 + 1:
                        z_mm = float(np.median(z_samples))

                        # Camera optical frame coordinates (meters)
                        x_cam = (cx_box - cx_cam) * z_mm / fx / 1000.0
                        y_cam = (cy_box - cy_cam) * z_mm / fy / 1000.0
                        z_cam = z_mm / 1000.0

                        # Transform to base_link
                        result = node.transform_point(x_cam, y_cam, z_cam)
                        if result:
                            bx, by, bz = result
                            horiz = math.sqrt(bx**2 + by**2) * 1000

                            # Overlay base_link coords on frame
                            cv2.putText(display,
                                       f'cam: X={x_cam*1000:.0f} Y={y_cam*1000:.0f} Z={z_mm:.0f}mm',
                                       (int(x1), int(y2) + 15),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                            cv2.putText(display,
                                       f'base: X={bx*1000:.0f} Y={by*1000:.0f} Z={bz*1000:.0f}mm',
                                       (int(x1), int(y2) + 30),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                            cv2.putText(display,
                                       f'dist: {horiz:.0f}mm',
                                       (int(x1), int(y2) + 45),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                            # Print to terminal (throttled)
                            if now - last_print > 1.0:
                                last_print = now
                                print(f'[{name}] conf={conf:.2f}')
                                print(f'  camera: X={x_cam*1000:.0f} Y={y_cam*1000:.0f} Z={z_mm:.0f} mm')
                                print(f'  base_link: X={bx*1000:.0f} Y={by*1000:.0f} Z={bz*1000:.0f} mm')
                                print(f'  horiz dist: {horiz:.0f} mm')
                                print(f'  burst: {len(z_samples)}/{burst_count} frames\n')

                elif not in_roi and len(depth_burst) >= burst_count:
                    cv2.putText(display, 'OUT OF RANGE',
                               (int(x1), int(y2) + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # HUD
            fps = 1.0 / max(time.time() - now, 0.001) if now > 0 else 0
            cv2.putText(display,
                       f'FPS: {fps:.1f} | Objects: {len(boxes)} | Burst: {burst_count}',
                       (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display,
                       f'Depth: {DEPTH_MIN_MM}-{DEPTH_MAX_MM}mm | Subpixel+ExtDisp',
                       (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            # Encode and serve MJPEG
            _, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with g_jpeg_lock:
                g_jpeg_frame = jpeg.tobytes()

            rclpy.spin_once(node, timeout_sec=0)


# ── Main ────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Verify camera-to-arm transform')
    parser.add_argument('--model', default='models/waste_yolo11n_v3_best.pt',
                        help='YOLO model path')
    parser.add_argument('--conf', type=float, default=0.35,
                        help='Confidence threshold')
    parser.add_argument('--port', type=int, default=8081,
                        help='MJPEG stream port')
    parser.add_argument('--burst', type=int, default=5,
                        help='Burst median frame count')
    parser.add_argument('--settle', type=float, default=0.3,
                        help='Settle delay after camera start (seconds)')
    parser.add_argument('--no-camera', action='store_true',
                        help='Manual mode: type XYZ instead of using camera')
    args = parser.parse_args()

    rclpy.init()
    node = TransformVerifier()

    if not node.wait_for_tf():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.print_transform_info()

    try:
        if args.no_camera:
            run_manual_mode(node)
        else:
            run_camera_mode(node, args.model, args.conf,
                          args.port, args.burst, args.settle)
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
