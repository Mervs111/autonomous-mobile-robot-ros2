#!/usr/bin/env python3
"""
goal_sender.py — Kirim Nav2 NavigateToPose goal dari terminal / topic.

Usage (terminal interaktif):
    ros2 run amr_slam goal_sender.py

Usage (satu goal langsung):
    ros2 run amr_slam goal_sender.py --ros-args \
        -p goal_x:=2.0 -p goal_y:=0.5 -p goal_yaw:=0.0 -p send_on_start:=true

Usage (via topic dari node lain):
    ros2 topic pub /goal_pose_2d std_msgs/msg/String \
        '{data: "x=2.0,y=0.5,yaw=0.0"}' --once
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

import math
import sys
import threading


def yaw_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)        # derajat
        self.declare_parameter('send_on_start', False)

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe /goal_pose_2d untuk trigger dari node lain
        self._sub = self.create_subscription(
            String, '/goal_pose_2d', self._topic_goal_callback,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))

        self.get_logger().info(
            '[goal_sender] Ready. Waiting for Nav2 action server...')
        self._client.wait_for_server()
        self.get_logger().info('[goal_sender] Nav2 action server connected.')

        if self.get_parameter('send_on_start').value:
            x = self.get_parameter('goal_x').value
            y = self.get_parameter('goal_y').value
            yaw = self.get_parameter('goal_yaw').value
            self.send_goal(x, y, yaw)

    def send_goal(self, x: float, y: float, yaw_deg: float = 0.0):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quaternion(yaw_deg)[2], yaw_to_quaternion(yaw_deg)[3]
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        self.get_logger().info(
            f'[goal_sender] Sending goal → x={x:.2f} y={y:.2f} yaw={yaw_deg:.1f}°')
        future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)

    def _topic_goal_callback(self, msg: String):
        # Parse format: "x=2.0,y=0.5,yaw=90.0"
        try:
            parts = {k: float(v) for k, v in
                     (item.split('=') for item in msg.data.split(','))}
            self.send_goal(parts.get('x', 0.0), parts.get('y', 0.0),
                           parts.get('yaw', 0.0))
        except Exception as e:
            self.get_logger().error(
                f'[goal_sender] Invalid /goal_pose_2d format: {msg.data} — {e}')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('[goal_sender] Goal REJECTED by Nav2.')
            return
        self.get_logger().info('[goal_sender] Goal ACCEPTED. Robot navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('[goal_sender] GOAL REACHED!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('[goal_sender] Goal CANCELED.')
        else:
            self.get_logger().error(
                f'[goal_sender] Goal FAILED (status={status}).')

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(
            f'[goal_sender] Distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)


def interactive_loop(node: GoalSender):
    """Baca goal dari stdin untuk mode interaktif."""
    print('\n[goal_sender] Interactive mode. Format: x y [yaw_deg]')
    print('  Contoh: 2.0 1.5       → ke (2.0, 1.5) yaw=0°')
    print('  Contoh: 2.0 1.5 90    → ke (2.0, 1.5) yaw=90°')
    print('  Ketik "q" untuk keluar.\n')
    while rclpy.ok():
        try:
            line = input('goal> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in ('q', 'quit', 'exit'):
            break
        parts = line.split()
        if len(parts) < 2:
            print('  Format salah. Butuh minimal: x y')
            continue
        try:
            x, y = float(parts[0]), float(parts[1])
            yaw = float(parts[2]) if len(parts) >= 3 else 0.0
            node.send_goal(x, y, yaw)
        except ValueError:
            print('  Nilai tidak valid.')


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()

    send_on_start = node.get_parameter('send_on_start').value
    if not send_on_start:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        interactive_loop(node)
    else:
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
