#!/usr/bin/env python3
"""
data_collector_node.py
=======================
Record dataset Visual Regression saat operator drive robot manual via joystick.

Subscribe (synchronized via ApproximateTimeSynchronizer):
    /camera/depth/image_rect_raw  (sensor_msgs/Image, 16UC1)
    /camera/color/image_raw       (sensor_msgs/Image, RGB)
Subscribe (latest, separate):
    /joy                           (sensor_msgs/Joy)
    /odom                          (nav_msgs/Odometry, optional)

Output struktur folder:
    datasets/run_<timestamp>/
        depth_000001.npy
        color_000001.jpg
        ...
        labels.csv

labels.csv kolom:
    frame_id, timestamp, depth_filename, color_filename,
    steering_cmd, velocity_cmd, x_pose, y_pose, theta

Usage:
    ros2 run amr_visual_regression data_collector_node \\
        --ros-args -p output_dir:=/home/azhar/datasets -p capture_rate:=10.0
"""
import os
import csv
import math
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import message_filters
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, Joy
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DataCollector(Node):

    def __init__(self):
        super().__init__('data_collector_node')

        # ---- Parameters ----
        self.declare_parameter('output_dir',     '/home/azhar/datasets')
        self.declare_parameter('capture_rate',   10.0)   # Hz
        self.declare_parameter('depth_topic',    '/camera/depth/image_rect_raw')
        self.declare_parameter('color_topic',    '/camera/color/image_raw')
        self.declare_parameter('joy_topic',      '/joy')
        self.declare_parameter('odom_topic',     '/odom')
        self.declare_parameter('axis_vel',       1)
        self.declare_parameter('axis_steer',     3)
        self.declare_parameter('deadman_button', 5)
        self.declare_parameter('record_only_when_deadman', True)

        gp = lambda n: self.get_parameter(n).value
        self.capture_period = 1.0 / max(gp('capture_rate'), 0.1)
        self.axis_vel = gp('axis_vel')
        self.axis_steer = gp('axis_steer')
        self.deadman_btn = gp('deadman_button')
        self.record_only_when_deadman = gp('record_only_when_deadman')

        # ---- Output folder ----
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(gp('output_dir'), f'run_{ts}')
        os.makedirs(self.run_dir, exist_ok=True)

        self.csv_path = os.path.join(self.run_dir, 'labels.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'frame_id', 'timestamp',
            'depth_filename', 'color_filename',
            'steering_cmd', 'velocity_cmd',
            'x_pose', 'y_pose', 'theta',
        ])

        self.get_logger().info(f'Output directory: {self.run_dir}')

        # ---- State ----
        self.bridge = CvBridge()
        self.frame_id = 0
        self.last_capture_t = 0.0
        self.latest_joy = None
        self.latest_odom = None

        # ---- Subscribers ----
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
        sub_depth = message_filters.Subscriber(self, Image, gp('depth_topic'), qos_profile=qos)
        sub_color = message_filters.Subscriber(self, Image, gp('color_topic'), qos_profile=qos)
        self.ts_sync = message_filters.ApproximateTimeSynchronizer(
            [sub_depth, sub_color], queue_size=10, slop=0.05
        )
        self.ts_sync.registerCallback(self.cb_synced)

        self.create_subscription(Joy, gp('joy_topic'),
                                 self.cb_joy, qos)
        self.create_subscription(Odometry, gp('odom_topic'),
                                 self.cb_odom, qos)

        self.get_logger().info(
            f'Data collector ready. Capture rate = {1.0/self.capture_period:.1f} Hz'
        )

    def cb_joy(self, msg: Joy):
        self.latest_joy = msg

    def cb_odom(self, msg: Odometry):
        self.latest_odom = msg

    def cb_synced(self, depth_msg: Image, color_msg: Image):
        now = time.time()
        if (now - self.last_capture_t) < self.capture_period:
            return

        # Joystick check (deadman + commands)
        steer_cmd, vel_cmd = 0.0, 0.0
        deadman = False
        if self.latest_joy is not None:
            joy = self.latest_joy
            if len(joy.buttons) > self.deadman_btn:
                deadman = (joy.buttons[self.deadman_btn] == 1)
            if len(joy.axes) > self.axis_vel:
                vel_cmd = float(joy.axes[self.axis_vel])
            if len(joy.axes) > self.axis_steer:
                steer_cmd = float(joy.axes[self.axis_steer])

        if self.record_only_when_deadman and not deadman:
            return

        # Pose from odom (kalau ada)
        x_pose = y_pose = theta = 0.0
        if self.latest_odom is not None:
            p = self.latest_odom.pose.pose
            x_pose = p.position.x
            y_pose = p.position.y
            theta = yaw_from_quat(p.orientation)

        # Convert images
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Save
        self.frame_id += 1
        depth_fn = f'depth_{self.frame_id:06d}.npy'
        color_fn = f'color_{self.frame_id:06d}.jpg'

        np.save(os.path.join(self.run_dir, depth_fn), depth_img)
        cv2.imwrite(os.path.join(self.run_dir, color_fn), color_img,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Append CSV
        ts_sec = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        self.csv_writer.writerow([
            self.frame_id, f'{ts_sec:.6f}',
            depth_fn, color_fn,
            f'{steer_cmd:.4f}', f'{vel_cmd:.4f}',
            f'{x_pose:.4f}', f'{y_pose:.4f}', f'{theta:.4f}',
        ])
        self.csv_file.flush()

        self.last_capture_t = now

        if self.frame_id % 50 == 0:
            self.get_logger().info(
                f'Captured frame #{self.frame_id} '
                f'(steer={steer_cmd:+.2f}, vel={vel_cmd:+.2f})'
            )

    def destroy_node(self):
        self.get_logger().info(
            f'Recording finished. Total frames = {self.frame_id}. '
            f'CSV: {self.csv_path}'
        )
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
