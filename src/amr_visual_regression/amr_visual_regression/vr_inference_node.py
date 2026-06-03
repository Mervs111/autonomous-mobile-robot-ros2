#!/usr/bin/env python3
"""
vr_inference_node.py
=====================
Real-time Visual Regression inference: depth -> (steering, velocity) -> /cmd_vel_visual.

Pipeline:
  1. Subscribe /camera/depth/image_rect_raw
  2. Extract features (modul feature_extractor)
  3. StandardScaler.transform()
  4. RandomForestRegressor.predict() -> (steering_norm, velocity_norm)
  5. Safety override: jika min_depth < safety_min_depth -> velocity = 0
  6. Publish /cmd_vel_visual (geometry_msgs/Twist) dalam konvensi Ackermann:
       linear.x  = velocity (m/s)
       angular.z = steering (rad), nanti dikonversi ke steering_angle di stm32_bridge

Note: model & scaler harus sudah ditraining offline pakai scripts/train.py.
"""
import math
import os
import time

import cv2
import joblib
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from amr_visual_regression.feature_extractor import extract_features


class VRInferenceNode(Node):

    def __init__(self):
        super().__init__('vr_inference_node')

        # ---- Parameters ----
        self.declare_parameter('model_path',     '/home/azhar/models/vr_model.pkl')
        self.declare_parameter('scaler_path',    '/home/azhar/models/vr_scaler.pkl')
        self.declare_parameter('depth_topic',    '/camera/depth/image_rect_raw')
        self.declare_parameter('cmd_topic',      '/cmd_vel_visual')
        self.declare_parameter('debug_topic',    '/vr_debug')
        self.declare_parameter('num_regions',     9)
        self.declare_parameter('roi_top',         200)
        self.declare_parameter('roi_bottom',      360)
        self.declare_parameter('free_threshold_m', 1.5)
        self.declare_parameter('max_depth_m',     6.0)
        self.declare_parameter('safety_min_depth', 0.4)
        self.declare_parameter('vx_max',          0.4)
        self.declare_parameter('steer_max_rad',   0.785)
        self.declare_parameter('publish_rate',    10.0)

        gp = lambda n: self.get_parameter(n).value
        self.num_regions = gp('num_regions')
        self.roi = (gp('roi_top'), gp('roi_bottom'))
        self.free_threshold_m = gp('free_threshold_m')
        self.max_depth_m = gp('max_depth_m')
        self.safety_min_depth = gp('safety_min_depth')
        self.vx_max = gp('vx_max')
        self.steer_max_rad = gp('steer_max_rad')

        # ---- Load model & scaler (gracefully handle missing files) ----
        model_path = gp('model_path')
        scaler_path = gp('scaler_path')

        if not os.path.exists(model_path) or not os.path.exists(scaler_path):
            self.get_logger().warn(
                f'Model or scaler not found at {model_path} / {scaler_path}.\n'
                f'Run training first: python3 scripts/train.py --dataset <path>'
            )
            self.model = None
            self.scaler = None
        else:
            self.model = joblib.load(model_path)
            self.scaler = joblib.load(scaler_path)
            self.get_logger().info(f'Loaded model: {model_path}')
            self.get_logger().info(f'Loaded scaler: {scaler_path}')

        # ---- ROS interface ----
        self.bridge = CvBridge()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

        self.sub_depth = self.create_subscription(
            Image, gp('depth_topic'), self.cb_depth, qos)
        self.pub_cmd = self.create_publisher(Twist, gp('cmd_topic'), 10)
        self.pub_dbg = self.create_publisher(String, gp('debug_topic'), 10)

        self.last_depth_t = 0.0
        self.publish_period = 1.0 / max(gp('publish_rate'), 1.0)

        self.get_logger().info('Visual Regression inference node ready.')

    def cb_depth(self, msg: Image):
        # Throttle to publish_rate
        now = time.time()
        if (now - self.last_depth_t) < self.publish_period:
            return
        self.last_depth_t = now

        if self.model is None or self.scaler is None:
            return

        # Convert depth image
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Pastikan uint16 (mm). Beberapa publisher pakai float32 (m).
        if depth.dtype == np.float32:
            depth_uint16 = (depth * 1000.0).astype(np.uint16)
        else:
            depth_uint16 = depth.astype(np.uint16)

        # Feature extraction
        feats = extract_features(
            depth_uint16,
            num_regions=self.num_regions,
            roi=self.roi,
            free_threshold_m=self.free_threshold_m,
            max_depth_m=self.max_depth_m,
        )

        # Predict
        feats_scaled = self.scaler.transform(feats.reshape(1, -1))
        pred = self.model.predict(feats_scaled)[0]  # [steering_norm, velocity_norm]

        steer_norm = float(np.clip(pred[0], -1.0, 1.0))
        vel_norm = float(np.clip(pred[1], -1.0, 1.0))

        # Convert normalized -> physical
        steering = steer_norm * self.steer_max_rad
        velocity = vel_norm * self.vx_max

        # Safety override: cek min_depth dari semua region
        min_depths = feats[1::4]  # index 1 dari setiap region (min_depth)
        valid_min = min_depths[min_depths > 0.01]
        global_min = float(valid_min.min()) if valid_min.size else float('nan')

        safety_triggered = False
        if not math.isnan(global_min) and global_min < self.safety_min_depth:
            velocity = 0.0
            safety_triggered = True

        # Publish cmd_vel
        out = Twist()
        out.linear.x = velocity
        out.angular.z = steering   # interpretasi: steering angle (rad), bukan yaw rate
        self.pub_cmd.publish(out)

        # Debug
        dbg = String()
        dbg.data = (
            f'{{"steering_rad": {steering:.3f}, '
            f'"velocity_mps": {velocity:.3f}, '
            f'"min_depth_m": {global_min:.3f}, '
            f'"safety_stop": {str(safety_triggered).lower()}}}'
        )
        self.pub_dbg.publish(dbg)


def main():
    rclpy.init()
    node = VRInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
