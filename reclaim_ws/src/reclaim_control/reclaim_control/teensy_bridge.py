"""
teensy_bridge.py — Unified serial bridge for RECLAIM robot.

Owns /dev/ttyACM0 (Teensy 4.1) and provides:
  1. /cmd_vel subscriber  → DRIVE serial command (differential drive)
  2. /odom publisher      → from encoder ticks (TICKS serial command)
  3. PTY proxy            → servo_hardware_interface connects to /tmp/teensy_arm
                            instead of the real serial port, so MoveIt2 and
                            driving can coexist on one Teensy.

Serial protocol (Teensy firmware must support these):
  DRIVE left right   → OK DRIVE        (signed -255..255 PWM)
  TICKS              → TICKS left right (encoder counts)
  RESET_TICKS        → OK RESET_TICKS
  ESTOP              → OK ESTOP
  (arm commands forwarded from PTY: SETUS, ARM, DISARM, etc.)
"""

import math
import os
import pty
import select
import sys
import termios
import threading
import time

# Force unbuffered output so logs appear in redirected files immediately
os.environ['PYTHONUNBUFFERED'] = '1'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

import serial


class TeensyBridge(Node):
    def __init__(self):
        super().__init__('teensy_bridge')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('pty_link_path', '/tmp/teensy_arm')
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('ticks_per_rev', 7560)
        self.declare_parameter('odom_rate', 20.0)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.pty_link_path = self.get_parameter('pty_link_path').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.odom_rate = self.get_parameter('odom_rate').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        # Derived: max wheel speed at motor's no-load RPM (37 RPM for JGB37-520)
        self.max_wheel_speed = self.wheel_radius * 2.0 * math.pi * 37.0 / 60.0

        # --- Serial connection ---
        self.serial_lock = threading.Lock()
        self.ser = None
        self.serial_error_count = 0
        self._open_serial()

        # --- PTY proxy for servo_hardware_interface ---
        self.pty_master_fd = None
        self.pty_slave_fd = None
        self._setup_pty()

        # --- Odometry state ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.ticks_initialized = False
        self.last_odom_time = self.get_clock().now()

        # Velocity estimates for odom message
        self.vx = 0.0
        self.vth = 0.0

        # --- cmd_vel state ---
        self.last_cmd_vel_time = None
        self.cmd_vel_active = False

        # --- Heading hold ---
        self.declare_parameter('heading_hold_kp', 2.0)  # P gain for heading correction
        self.heading_hold_kp = self.get_parameter('heading_hold_kp').value
        self.heading_hold_target = None  # locked heading when driving straight

        # --- Ramp limiter state ---
        self.declare_parameter('max_acceleration', 0.3)  # m/s²
        self.declare_parameter('use_closed_loop', True)   # SETVEL vs DRIVE
        self.max_accel = self.get_parameter('max_acceleration').value
        self.use_closed_loop = self.get_parameter('use_closed_loop').value
        self.current_v_left = 0.0
        self.current_v_right = 0.0

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscribers ---
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # --- Timers ---
        odom_period = 1.0 / self.odom_rate
        self.create_timer(odom_period, self._odom_timer_callback)
        self.create_timer(0.1, self._cmd_vel_timeout_callback)

        # --- PTY forwarding thread ---
        self.pty_thread_running = True
        self.pty_thread = threading.Thread(target=self._pty_forward_loop, daemon=True)
        self.pty_thread.start()

        # --- Reset encoder ticks ---
        self._send_serial('RESET_TICKS', expect_prefix='OK')
        self.odom_ok_count = 0
        self.odom_fail_count = 0

        # Startup diagnostic: verify TICKS works
        test_ticks = self._send_serial('TICKS', expect_prefix='TICKS')
        if test_ticks.startswith('TICKS'):
            self.get_logger().info(f'Encoder test OK: {test_ticks}')
        else:
            self.get_logger().warn(f'Encoder test FAILED (got: "{test_ticks}") — odometry will not work!')

        self.get_logger().info(
            f'TeensyBridge started: serial={self.serial_port}, '
            f'pty={self.pty_link_path}, '
            f'wheel_sep={self.wheel_separation}m, '
            f'wheel_r={self.wheel_radius}m, '
            f'ticks/rev={self.ticks_per_rev}'
        )

    # ------------------------------------------------------------------
    # Serial I/O
    # ------------------------------------------------------------------

    def _open_serial(self):
        """Open the real Teensy serial port."""
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.05,
            )
            time.sleep(2.0)  # Wait for Teensy boot
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Serial port {self.serial_port} opened')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

    def _reconnect_serial(self):
        """Attempt to reconnect to Teensy serial port."""
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.serial_error_count = 0
        time.sleep(1.0)
        self._open_serial()
        if self.ser is not None:
            self.get_logger().info('Serial reconnected successfully')

    def _send_serial(self, cmd: str, expect_prefix: str = '') -> str:
        """Send a command to Teensy and return the response line (thread-safe).

        Args:
            cmd: Command string to send (e.g. 'TICKS', 'SETVEL 0.1 0.1')
            expect_prefix: If set, keep reading lines until one starts with this
                           prefix (or timeout). This prevents SETVEL responses
                           from being returned when we asked for TICKS.
        """
        if self.ser is None:
            self._reconnect_serial()
            if self.ser is None:
                return ''
        with self.serial_lock:
            try:
                self.ser.write((cmd.strip() + '\n').encode('utf-8'))
                # Read response lines until we get the expected one or timeout
                deadline = time.monotonic() + 0.25
                while time.monotonic() < deadline:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.serial_error_count = 0
                        # If we have an expected prefix, skip non-matching lines
                        if expect_prefix and not line.startswith(expect_prefix):
                            continue
                        return line
                # No response — count as soft error
                self.serial_error_count = getattr(self, 'serial_error_count', 0) + 1
                if self.serial_error_count > 20:
                    self.get_logger().warn('Too many serial timeouts — reconnecting...')
                    self.ser.close()
                    self.ser = None
                return ''
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                self.ser.close()
                self.ser = None
                return ''

    def _send_serial_raw(self, data: bytes):
        """Write raw bytes to Teensy serial (thread-safe). Used by PTY forwarder."""
        if self.ser is None:
            return
        with self.serial_lock:
            try:
                self.ser.write(data)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _read_serial_available(self) -> bytes:
        """Read any available bytes from Teensy serial (thread-safe)."""
        if self.ser is None:
            return b''
        with self.serial_lock:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    return self.ser.read(waiting)
                return b''
            except serial.SerialException:
                return b''

    # ------------------------------------------------------------------
    # PTY proxy for servo_hardware_interface
    # ------------------------------------------------------------------

    def _setup_pty(self):
        """Create a pseudo-terminal and symlink it for servo_hardware_interface."""
        self.pty_master_fd, self.pty_slave_fd = pty.openpty()
        slave_name = os.ttyname(self.pty_slave_fd)

        # Configure PTY slave to match real serial settings (raw mode, 115200)
        attrs = termios.tcgetattr(self.pty_slave_fd)
        # Set baud rate
        attrs[4] = termios.B115200  # ispeed
        attrs[5] = termios.B115200  # ospeed
        # Raw mode
        attrs[0] = 0  # iflag
        attrs[1] = 0  # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
        attrs[3] = 0  # lflag
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(self.pty_slave_fd, termios.TCSANOW, attrs)

        # Create symlink (remove old one if exists)
        if os.path.islink(self.pty_link_path) or os.path.exists(self.pty_link_path):
            os.remove(self.pty_link_path)
        os.symlink(slave_name, self.pty_link_path)

        self.get_logger().info(
            f'PTY proxy created: {slave_name} -> {self.pty_link_path}'
        )

    def _pty_forward_loop(self):
        """Background thread: forward commands from PTY master to real serial.

        Only forwards PTY→serial (arm commands from servo_hardware_interface).
        Does NOT read from serial→PTY, because that steals TICKS/SETVEL
        responses needed by the bridge's own odom and drive logic.
        """
        while self.pty_thread_running:
            try:
                # Check if PTY master has data to read (arm commands)
                readable, _, _ = select.select([self.pty_master_fd], [], [], 0.05)
                if self.pty_master_fd in readable:
                    data = os.read(self.pty_master_fd, 4096)
                    if data:
                        self._send_serial_raw(data)

            except OSError as e:
                if e.errno == 5:  # EIO — slave side closed
                    time.sleep(0.1)
                else:
                    self.get_logger().error(f'PTY forward error: {e}')
                    time.sleep(0.5)

    # ------------------------------------------------------------------
    # /cmd_vel → SETVEL command (closed-loop) or DRIVE (open-loop)
    # ------------------------------------------------------------------

    def _ramp_limit(self, current: float, target: float, dt: float) -> float:
        """Apply acceleration limit to a velocity value.

        Prevents jerky starts/stops by limiting how fast velocity can change.
        """
        if dt <= 0:
            return target
        max_delta = self.max_accel * dt
        if target > current:
            return min(target, current + max_delta)
        else:
            return max(target, current - max_delta)

    def _cmd_vel_callback(self, msg: Twist):
        """Convert Twist to wheel velocities and send to Teensy.

        Uses SETVEL (closed-loop PI on Teensy) or DRIVE (open-loop PWM)
        depending on use_closed_loop parameter.

        Heading hold: when driving straight (angular.z ≈ 0 and linear.x != 0),
        locks the current heading and applies a P correction to keep the robot
        on course despite castor drag or wheel speed mismatch.
        """
        v = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        w = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

        # Heading hold: correct drift when driving straight
        if abs(v) > 0.005 and abs(w) < 0.02:
            # Driving straight — lock heading on first callback
            if self.heading_hold_target is None:
                self.heading_hold_target = self.theta
            # Compute heading error (wrapped to [-pi, pi])
            heading_error = math.atan2(
                math.sin(self.heading_hold_target - self.theta),
                math.cos(self.heading_hold_target - self.theta))
            w = self.heading_hold_kp * heading_error
        else:
            # Turning or stopped — release heading lock
            self.heading_hold_target = None

        # Differential drive inverse kinematics
        target_v_left = v - w * self.wheel_separation / 2.0
        target_v_right = v + w * self.wheel_separation / 2.0

        # Apply ramp limiter for smooth acceleration
        dt = 1.0 / self.odom_rate  # approximate dt from odom rate
        self.current_v_left = self._ramp_limit(
            self.current_v_left, target_v_left, dt)
        self.current_v_right = self._ramp_limit(
            self.current_v_right, target_v_right, dt)

        if self.use_closed_loop:
            # Closed-loop: send target velocity to Teensy PI controller
            self._send_serial(
                f'SETVEL {self.current_v_left:.4f} {self.current_v_right:.4f}')
        else:
            # Open-loop fallback: convert velocity to PWM
            if self.max_wheel_speed > 0:
                left_pwm = int(max(-255, min(255,
                    self.current_v_left / self.max_wheel_speed * 255.0)))
                right_pwm = int(max(-255, min(255,
                    self.current_v_right / self.max_wheel_speed * 255.0)))
            else:
                left_pwm = 0
                right_pwm = 0
            self._send_serial(f'DRIVE {left_pwm} {right_pwm}')

        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_active = True

    def _cmd_vel_timeout_callback(self):
        """Stop motors if no cmd_vel received recently."""
        if not self.cmd_vel_active:
            return
        if self.last_cmd_vel_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            if self.use_closed_loop:
                self._send_serial('SETVEL 0.0 0.0')
            else:
                self._send_serial('DRIVE 0 0')
            self.current_v_left = 0.0
            self.current_v_right = 0.0
            self.cmd_vel_active = False

    # ------------------------------------------------------------------
    # Odometry from encoder ticks
    # ------------------------------------------------------------------

    def _odom_timer_callback(self):
        """Poll TICKS from Teensy, compute and publish odometry."""
        response = self._send_serial('TICKS', expect_prefix='TICKS')
        if not response.startswith('TICKS'):
            self.odom_fail_count += 1
            # Log first failure immediately, then every 20 (~2s at 10Hz)
            if self.odom_fail_count <= 3 or self.odom_fail_count % 20 == 0:
                self.get_logger().warn(
                    f'TICKS fail #{self.odom_fail_count} (got: "{response[:60]}") '
                    f'serial={self.ser is not None}')
            return
        self.odom_ok_count += 1
        # Log first 3 successes, then every 100 (~10s at 10Hz)
        if self.odom_ok_count <= 3 or self.odom_ok_count % 100 == 0:
            self.get_logger().info(
                f'Odom OK #{self.odom_ok_count}: theta={math.degrees(self.theta):.1f}deg '
                f'x={self.x:.3f}m y={self.y:.3f}m | {response}')

        parts = response.split()
        if len(parts) != 3:
            return

        try:
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])
        except ValueError:
            return

        now = self.get_clock().now()

        # Initialize on first reading
        if not self.ticks_initialized:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_odom_time = now
            self.ticks_initialized = True
            return

        # Compute deltas
        dl_ticks = left_ticks - self.prev_left_ticks
        dr_ticks = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0:
            return

        # Convert ticks to distance
        # Encoders count negative for forward motion, so negate
        meters_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
        d_left = -dl_ticks * meters_per_tick
        d_right = -dr_ticks * meters_per_tick

        # Differential drive odometry
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # Update pose (midpoint integration)
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocity estimates
        self.vx = d_center / dt
        self.vth = d_theta / dt

        # Publish odometry
        self._publish_odom(now)

    def _publish_odom(self, stamp):
        """Publish Odometry message and TF transform."""
        # Quaternion from yaw
        q = self._yaw_to_quaternion(self.theta)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        # Covariance — rough values, can tune later
        # Position covariance (x, y, z, rot_x, rot_y, rot_z) — 6x6 row-major
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # yaw
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.03 # vyaw

        self.odom_pub.publish(odom)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert a yaw angle (radians) to a Quaternion message."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.pty_thread_running = False
        if self.pty_thread.is_alive():
            self.pty_thread.join(timeout=1.0)

        # Stop motors
        if self.ser:
            self._send_serial('DRIVE 0 0')

        # Clean up PTY
        if self.pty_link_path and os.path.islink(self.pty_link_path):
            os.remove(self.pty_link_path)
        if self.pty_master_fd is not None:
            os.close(self.pty_master_fd)
        if self.pty_slave_fd is not None:
            os.close(self.pty_slave_fd)

        # Close serial
        if self.ser and self.ser.is_open:
            self.ser.close()

        super().destroy_node()


def main(args=None):
    # Ensure unbuffered output for log redirection
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)
    sys.stderr = os.fdopen(sys.stderr.fileno(), 'w', buffering=1)

    rclpy.init(args=args)
    node = TeensyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
