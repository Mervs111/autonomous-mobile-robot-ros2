#!/usr/bin/env python3
"""
amr_loop_patrol.py — PLAN B: Robot keliling loop SENDIRI pakai WHEEL ODOMETRY.

TIDAK butuh peta / VIO / localization / Nav2. Selama encoder roda (/odom) jalan,
robot bisa mengelilingi ruangan & balik ke titik awal (perkiraan). Robust untuk
ruangan polos di mana VIO sering lost.

Cara kerja: state machine baca /odom, eksekusi rute segmen demi segmen:
  - F (Forward): maju lurus N meter (setir 0)
  - T (Turn)   : belok arc Ackermann sampai yaw berubah N derajat
Robot Ackermann TIDAK bisa putar di tempat -> belok = arc (maju + setir penuh).

PRASYARAT:
  1. Terminal sensor jalan:
       ros2 launch amr_bringup amr_full.launch.py use_slam:=false use_nav2:=false \
            use_rtabmap:=false use_vr:=false use_failover:=false
     (ini menghidupkan /odom dari encoder + stm32_bridge)
  2. Aktifkan autonomous (forward-only sudah default):
       ros2 param set /stm32_bridge autonomous_enabled true

JALANKAN:
  ros2 run amr_slam amr_loop_patrol.py
  # rute custom (F=maju meter, T=belok derajat; + kiri, - kanan):
  ros2 run amr_slam amr_loop_patrol.py --ros-args \
      -p route:="F:2.0; T:90; F:1.5; T:90; F:2.0; T:90; F:1.5; T:90"

STOP DARURAT: Ctrl+C (kirim stop) · pegang R1 (manual override menang) ·
  ros2 param set /stm32_bridge autonomous_enabled false
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def ang_diff(a, b):
    """selisih sudut a-b ternormalisasi ke [-pi, pi]"""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class LoopPatrol(Node):
    def __init__(self):
        super().__init__('amr_loop_patrol')

        # Rute default = persegi panjang ~2x1.5m. SESUAIKAN dengan ruanganmu!
        self.declare_parameter('route',
                               'F:2.0; T:90; F:1.5; T:90; F:2.0; T:90; F:1.5; T:90')
        self.declare_parameter('fwd_speed', 0.15)    # m/s saat maju lurus
        self.declare_parameter('turn_speed', 0.12)   # m/s saat belok (arc)
        self.declare_parameter('min_radius', 0.90)   # m, radius belok Ackermann
        self.declare_parameter('seg_timeout', 60.0)  # detik max/segmen (safety)

        self.fwd_speed = float(self.get_parameter('fwd_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.min_radius = float(self.get_parameter('min_radius').value)
        self.seg_timeout = float(self.get_parameter('seg_timeout').value)
        self.segments = self._parse_route(self.get_parameter('route').value)

        if not self.segments:
            self.get_logger().error('Rute kosong/invalid. Cek parameter route.')
            raise SystemExit(1)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 20)

        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.idx = 0
        self.seg_started = False
        self.done = False
        self.seg_start_x = self.seg_start_y = 0.0
        self.seg_yaw_accum = 0.0
        self.prev_yaw = 0.0
        self.t_seg_start = self.get_clock().now()

        self.get_logger().info(
            f'[loop_patrol] {len(self.segments)} segmen siap. Menunggu /odom...')
        self.timer = self.create_timer(0.05, self._control)   # 20 Hz

    def _parse_route(self, raw):
        segs = []
        for chunk in str(raw).split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            try:
                kind, val = chunk.split(':')
                kind = kind.strip().upper()
                if kind in ('F', 'T'):
                    segs.append((kind, float(val)))
            except ValueError:
                self.get_logger().warn(f'Segmen invalid, dilewati: "{chunk}"')
        return segs

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        if not self.have_odom:
            self.prev_yaw = self.yaw
            self.have_odom = True

    def _stop(self):
        self.pub.publish(Twist())

    def _next_segment(self):
        self.idx += 1
        self.seg_started = False

    def _control(self):
        if not self.have_odom:
            return

        if self.idx >= len(self.segments):
            self._stop()
            if not self.done:
                self.done = True
                self.timer.cancel()
                self.get_logger().info(
                    '[loop_patrol] SELESAI — robot kembali ke titik awal '
                    '(perkiraan). Ctrl+C untuk keluar.')
            return

        kind, target = self.segments[self.idx]

        if not self.seg_started:
            self.seg_start_x, self.seg_start_y = self.x, self.y
            self.prev_yaw = self.yaw
            self.seg_yaw_accum = 0.0
            self.t_seg_start = self.get_clock().now()
            self.seg_started = True
            self.get_logger().info(
                f'[loop_patrol] Segmen {self.idx + 1}/{len(self.segments)} '
                f'-> {kind}:{target}')

        # safety timeout
        if (self.get_clock().now() - self.t_seg_start).nanoseconds / 1e9 > self.seg_timeout:
            self.get_logger().warn('[loop_patrol] Segmen TIMEOUT — lanjut berikutnya.')
            self._stop()
            self._next_segment()
            return

        cmd = Twist()
        if kind == 'F':
            dist = math.hypot(self.x - self.seg_start_x, self.y - self.seg_start_y)
            self.get_logger().info(f'  maju {dist:.2f}/{target:.2f} m',
                                   throttle_duration_sec=1.0)
            if dist >= target:
                self._stop()
                self._next_segment()
                return
            cmd.linear.x = self.fwd_speed
        else:  # 'T' belok arc Ackermann
            self.seg_yaw_accum += ang_diff(self.yaw, self.prev_yaw)
            self.prev_yaw = self.yaw
            deg = math.degrees(self.seg_yaw_accum)
            self.get_logger().info(f'  belok {deg:.0f}/{target:.0f} deg',
                                   throttle_duration_sec=1.0)
            if abs(deg) >= abs(target):
                self._stop()
                self._next_segment()
                return
            sign = 1.0 if target >= 0 else -1.0       # + kiri, - kanan
            cmd.linear.x = self.turn_speed
            cmd.angular.z = sign * (self.turn_speed / self.min_radius)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LoopPatrol()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            if node is not None:
                node.pub.publish(Twist())   # pastikan robot STOP
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
