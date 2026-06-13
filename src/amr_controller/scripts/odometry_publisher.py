#!/usr/bin/env python3
"""
================================================================
  odometry_publisher.py
  Wheel-only Odometry Publisher for Mobile Robot Ackermann Indoor

  Subscribes:
    /encoder       (std_msgs/Int32)  - encoder reading from STM32
    /joy           (sensor_msgs/Joy) - to read steering command

  Publishes:
    /odom          (nav_msgs/Odometry)
    TF broadcaster: odom -> base_footprint

  Math: Bicycle kinematic model (Ackermann)
    distance_per_tick = 2*pi*wheel_radius / pulses_per_revolution
    vx     = delta_distance / dt
    delta  = steering_angle (rad), from /joy axis 3 mapped to ±45°
    x      += vx * cos(theta) * dt
    y      += vx * sin(theta) * dt
    theta  += (vx/wheelbase) * tan(delta) * dt

  AUTO-DETECT encoder format:
    If first 5 readings are monotonically increasing → CUMULATIVE
    Else → DELTA per interval

  IMPORTANT: Tanpa IMU, yaw drift akan signifikan. SLAM Toolbox
  scan-matching akan mengoreksi via ICP. Output odometry ini
  cukup sebagai motion prior untuk SLAM, BUKAN sebagai source of
  truth standalone.

  Author: Muhammad Al Azhar Faradis (NRP 2040241017)
  ITS Surabaya - Departemen Teknik Elektro Otomasi
================================================================
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Yaw (rad) -> geometry_msgs/Quaternion (only z and w needed)."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdometryPublisher(Node):

    # Auto-detect history size
    DETECT_HISTORY = 5

    def __init__(self):
        super().__init__('odometry_publisher')

        # ========== Parameters ==========
        self.declare_parameter('wheel_radius', 0.0775)        # 155 mm / 2
        self.declare_parameter('wheelbase', 0.500)             # m
        self.declare_parameter('pulses_per_revolution', 1496)  # 11 PPR × 4 quad × 1:34
        self.declare_parameter('max_steer_deg', 45.0)
        self.declare_parameter('publish_rate', 50.0)           # Hz
        self.declare_parameter('joy_steer_axis', 3)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('encoder_format', 'auto')       # 'auto', 'delta', 'cumulative'
        # KALIBRASI jarak: odom ternyata ~2x lebih besar dari fisik (F:1.0 -> ~50cm).
        # dist_scale=0.5 mengoreksi. TUNABLE LIVE (tanpa rebuild):
        #   ros2 param set /odometry_publisher dist_scale 0.5
        self.declare_parameter('dist_scale', 0.5)

        gp = self.get_parameter
        self.wheel_radius = gp('wheel_radius').value
        self.wheelbase = gp('wheelbase').value
        self.ppr = gp('pulses_per_revolution').value
        self.max_steer = math.radians(gp('max_steer_deg').value)
        self.rate = gp('publish_rate').value
        self.joy_steer_axis = gp('joy_steer_axis').value
        self.odom_frame = gp('odom_frame').value
        self.base_frame = gp('base_frame').value
        self.publish_tf_flag = gp('publish_tf').value
        self.encoder_format = gp('encoder_format').value

        # Distance per encoder tick (meter)
        self.dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ppr

        # ========== State ==========
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.steering = 0.0  # current steering command (rad)
        # waktu cmd_vel terakhir (init di masa lalu agar joystick aktif saat awal)
        self.last_cmd_vel_t = self.get_clock().now() - Duration(seconds=10)

        # Encoder state
        self.last_encoder = None      # last raw value
        self.last_delta = 0           # last computed delta ticks
        self.encoder_history = deque(maxlen=self.DETECT_HISTORY)
        self.detected_format = self.encoder_format if self.encoder_format != 'auto' else None

        # ========== I/O ==========
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub_enc = self.create_subscription(
            Int32, '/encoder', self.encoder_cb, qos)
        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_cb, qos)
        # FIX: baca setir juga dari /cmd_vel (mode autonomous). Tanpa ini, yaw
        # odom TIDAK PERNAH berubah saat robot jalan via cmd_vel (joystick netral)
        # -> patrol mentok "belok 0/90". Lihat cmd_vel_cb.
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.pub_odom = self.create_publisher(Odometry, '/odom', qos)

        if self.publish_tf_flag:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing at fixed rate
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info(
            f'Odometry Publisher started:\n'
            f'  wheel_radius   = {self.wheel_radius:.4f} m\n'
            f'  wheelbase      = {self.wheelbase:.4f} m\n'
            f'  PPR            = {self.ppr}\n'
            f'  dist_per_tick  = {self.dist_per_tick*1000:.4f} mm\n'
            f'  encoder_format = {self.encoder_format}'
        )

    # =====================================================
    # Callbacks
    # =====================================================
    def joy_cb(self, msg: Joy):
        """Setir dari joystick — HANYA saat tidak ada cmd_vel autonomous aktif."""
        # Kalau cmd_vel baru diterima (<0.5s) = mode autonomous -> jangan timpa
        # dengan joystick netral (yang bikin steering balik ke 0).
        dt_cmd = (self.get_clock().now() - self.last_cmd_vel_t).nanoseconds / 1e9
        if dt_cmd < 0.5:
            return
        if len(msg.axes) > self.joy_steer_axis:
            steer_raw = msg.axes[self.joy_steer_axis]  # -1.0 to 1.0
            # Match stm32_bridge convention (negate for hardware right turn)
            self.steering = -steer_raw * self.max_steer

    def cmd_vel_cb(self, msg: Twist):
        """Mode autonomous: turunkan sudut setir dari /cmd_vel (Ackermann inverse).
        steer = atan(wheelbase * angular.z / linear.x). Ini yang bikin yaw odom
        berubah saat robot belok otonom (sebelumnya selalu 0)."""
        v = msg.linear.x
        w = msg.angular.z
        if abs(v) > 0.03:
            self.steering = math.atan(self.wheelbase * w / v)
        else:
            self.steering = 0.0
        self.last_cmd_vel_t = self.get_clock().now()

    def encoder_cb(self, msg: Int32):
        """Process encoder reading; auto-detect cumulative vs delta."""
        value = msg.data
        self.encoder_history.append(value)

        # Auto-detect format on first DETECT_HISTORY samples
        if self.detected_format is None:
            if len(self.encoder_history) >= self.DETECT_HISTORY:
                # Check if monotonically increasing (cumulative) or oscillating around 0 (delta)
                vals = list(self.encoder_history)
                diffs = [vals[i+1] - vals[i] for i in range(len(vals)-1)]
                # If most diffs are large and same sign → cumulative
                # If diffs are smaller magnitude and similar to original values → delta
                avg_val = sum(abs(v) for v in vals) / len(vals)
                avg_diff = sum(abs(d) for d in diffs) / len(diffs)

                if avg_val > avg_diff * 5:
                    self.detected_format = 'cumulative'
                else:
                    self.detected_format = 'delta'

                self.get_logger().info(
                    f'[AUTO-DETECT] Encoder format: {self.detected_format} '
                    f'(avg_val={avg_val:.1f}, avg_diff={avg_diff:.1f})'
                )

        # Compute delta based on detected format
        if self.detected_format == 'cumulative':
            if self.last_encoder is None:
                self.last_encoder = value
                self.last_delta = 0
            else:
                self.last_delta = value - self.last_encoder
                self.last_encoder = value
        else:  # 'delta'
            self.last_delta = value

    # =====================================================
    # Update odometry & publish
    # =====================================================
    def update(self):
        if self.detected_format is None:
            return  # not enough samples yet

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # Distance traveled this tick (× dist_scale untuk kalibrasi 2x)
        delta_dist = self.last_delta * self.dist_per_tick * self.get_parameter('dist_scale').value
        # Reset delta after consuming (only relevant for 'delta' format)
        # For 'cumulative', last_delta naturally reflects new readings
        if self.detected_format == 'delta':
            self.last_delta = 0

        vx = delta_dist / dt

        # Bicycle kinematic update
        # Use steering angle from joy callback
        delta_theta = (vx / self.wheelbase) * math.tan(self.steering) * dt

        # Integrate pose (Euler forward; OK at 50 Hz)
        self.x += delta_dist * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_dist * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Wrap theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ===== Publish /odom =====
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = delta_theta / dt if dt > 0 else 0.0

        # Covariance (rough estimate; high because no IMU correction)
        # Order: x, y, z, rot_x, rot_y, rot_z
        # Pose covariance (6x6 diagonal): xyz + roll/pitch/yaw
        odom.pose.covariance[0]  = 0.05      # x  (5 cm std)
        odom.pose.covariance[7]  = 0.05      # y
        odom.pose.covariance[14] = 99999.0   # z (unobserved, ground robot)
        odom.pose.covariance[21] = 99999.0   # roll (unobserved)
        odom.pose.covariance[28] = 99999.0   # pitch (unobserved)
        odom.pose.covariance[35] = 0.10      # yaw (high without IMU)
        # Twist covariance: vx vy vz, vroll vpitch vyaw
        odom.twist.covariance[0]  = 0.02     # vx
        odom.twist.covariance[7]  = 99999.0  # vy (Ackermann: no lateral)
        odom.twist.covariance[14] = 99999.0  # vz (unobserved)
        odom.twist.covariance[21] = 99999.0  # vroll (unobserved)
        odom.twist.covariance[28] = 99999.0  # vpitch (unobserved)
        odom.twist.covariance[35] = 0.05     # vyaw

        self.pub_odom.publish(odom)

        # ===== Publish TF =====
        if self.publish_tf_flag:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
