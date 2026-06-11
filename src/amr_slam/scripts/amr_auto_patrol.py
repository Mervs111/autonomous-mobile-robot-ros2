#!/usr/bin/env python3
"""
amr_auto_patrol.py — Robot jalan SENDIRI menyusuri daftar waypoint (tanpa joystick).

Mengirim goal Nav2 (NavigateToPose) satu per satu. Setelah satu goal tercapai,
otomatis lanjut ke goal berikutnya. Bisa loop terus atau sekali jalan.

Prasyarat (harus sudah jalan):
  1. Terminal sensor  : amr_full.launch.py (sensors)
  2. Localization     : rtabmap_localization (TF map->odom hidup)
  3. Nav2             : use_nav2:=true (action server /navigate_to_pose aktif)
  4. stm32_bridge     : autonomous_enabled=true
     -> ros2 param set /stm32_bridge autonomous_enabled true

Cara pakai:
  # waypoint default (EDIT dulu sesuai petamu! ambil koordinat dari RViz):
  ros2 run amr_slam amr_auto_patrol.py

  # waypoint custom via parameter (format: "x,y,yaw_deg; x,y,yaw_deg; ...")
  ros2 run amr_slam amr_auto_patrol.py --ros-args \
      -p waypoints:="0.0,0.0,0; 2.0,0.5,90; 2.0,2.0,180" -p loop:=false

  # berhenti darurat: Ctrl+C node ini (goal di-cancel), atau pegang R1
  # (manual override), atau:
  #   ros2 param set /stm32_bridge autonomous_enabled false

Cara ambil koordinat waypoint:
  Buka RViz (peta localization tampil) -> klik "Publish Point" di toolbar ->
  klik titik di peta -> baca koordinat di topic /clicked_point:
    ros2 topic echo /clicked_point --once
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


def yaw_to_quat(yaw_deg: float):
    y = math.radians(yaw_deg) / 2.0
    return math.sin(y), math.cos(y)   # (qz, qw)


class AutoPatrol(Node):
    def __init__(self):
        super().__init__('amr_auto_patrol')

        # ---- Parameter ----
        # waypoints: "x,y,yaw_deg; x,y,yaw_deg; ..."
        # DEFAULT INI CONTOH — WAJIB disesuaikan dengan peta lab kalian!
        self.declare_parameter('waypoints', '1.0,0.0,0; 1.0,1.0,90; 0.0,1.0,180; 0.0,0.0,270')
        self.declare_parameter('loop', True)           # ulangi rute terus?
        self.declare_parameter('pause_sec', 2.0)       # jeda antar waypoint

        self.waypoints = self._parse_waypoints(
            self.get_parameter('waypoints').value)
        self.loop = self.get_parameter('loop').value
        self.pause_sec = float(self.get_parameter('pause_sec').value)

        if not self.waypoints:
            self.get_logger().error('Tidak ada waypoint valid! Cek parameter.')
            raise SystemExit(1)

        self.idx = 0
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            f'[patrol] {len(self.waypoints)} waypoint, loop={self.loop}. '
            'Menunggu Nav2 action server...')
        self.client.wait_for_server()
        self.get_logger().info('[patrol] Nav2 siap. Mulai patroli!')
        self._send_next()

    # ------------------------------------------------------------
    def _parse_waypoints(self, raw: str):
        pts = []
        for chunk in raw.split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            try:
                vals = [float(v) for v in chunk.split(',')]
                x, y = vals[0], vals[1]
                yaw = vals[2] if len(vals) > 2 else 0.0
                pts.append((x, y, yaw))
            except (ValueError, IndexError):
                self.get_logger().warn(f'Waypoint tidak valid, dilewati: "{chunk}"')
        return pts

    # ------------------------------------------------------------
    def _send_next(self):
        x, y, yaw = self.waypoints[self.idx]
        qz, qw = yaw_to_quat(yaw)

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose

        self.get_logger().info(
            f'[patrol] Waypoint {self.idx + 1}/{len(self.waypoints)} '
            f'-> x={x:.2f} y={y:.2f} yaw={yaw:.0f}°')
        fut = self.client.send_goal_async(
            goal, feedback_callback=self._feedback)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('[patrol] Goal DITOLAK Nav2 — coba waypoint berikutnya.')
            self._advance()
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('[patrol] ✔ Waypoint TERCAPAI.')
        else:
            self.get_logger().warn(
                f'[patrol] ✘ Waypoint gagal (status={status}) — lanjut berikutnya.')
        self._advance()

    def _advance(self):
        self.idx += 1
        if self.idx >= len(self.waypoints):
            if self.loop:
                self.idx = 0
                self.get_logger().info('[patrol] Rute selesai — ULANGI dari awal (loop).')
            else:
                self.get_logger().info('[patrol] Rute selesai. PATROLI BERAKHIR.')
                rclpy.shutdown()
                return
        # jeda sebentar sebelum goal berikutnya (biar transisi halus)
        self._pause_timer = self.create_timer(self.pause_sec, self._next_once)

    def _next_once(self):
        # one-shot: matikan timer dulu, baru kirim goal berikutnya
        self._pause_timer.cancel()
        self._send_next()

    def _feedback(self, fb):
        d = fb.feedback.distance_remaining
        self.get_logger().info(f'[patrol] sisa jarak: {d:.2f} m',
                               throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AutoPatrol()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
