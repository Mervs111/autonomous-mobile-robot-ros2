#!/usr/bin/env python3
"""
amr_perception_node.py
======================
ROS 2 Node — integrasi pipeline PCD (dari amr_pcd_prototype.py) ke sistem AMR.
DRAFT pra-integrasi: jalankan SETELAH parameter divalidasi via prototipe standalone.

Subscribe : /camera/camera/color/image_raw (sensor_msgs/Image, QoS BEST_EFFORT)
Publish   : /cmd_vel_visual (geometry_msgs/Twist)  -> arbitrasi via amr_failover
            /pcd_debug/image (sensor_msgs/Image)   -> visualisasi deteksi

Pipeline sama persis dengan prototipe: ROI crop -> resize -> blur -> Otsu ->
morfologi -> contour -> zona navigasi (Aman/Peringatan/Bahaya).

Jalankan:
  source ~/amr_starter/install/setup.bash
  python3 amr_perception_node.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np

# ===== Parameter pipeline (samakan dengan hasil tuning prototipe standalone) =====
TARGET_WIDTH = 640
ROI_CROP_RATIO = 0.60
BLUR_KERNEL = (5, 5)
MORPH_KERNEL = (5, 5)
MIN_AREA = 500

ZONE_WARN = 8000
ZONE_DANGER = 20000

# ===== Perintah gerak per zona (konservatif untuk Ackermann) =====
SPEED_SAFE = 0.20     # m/s  — zona aman: maju pelan
SPEED_WARN = 0.10     # m/s  — zona peringatan: sangat pelan
STEER_WARN = 0.30     # rad/s — belok menjauhi obstacle
SPEED_STOP = 0.0      # zona bahaya: berhenti


class AmrPerceptionNode(Node):
    def __init__(self):
        super().__init__('amr_perception_node')
        self.bridge = CvBridge()

        # QoS BEST_EFFORT — cocok untuk stream kamera (boleh drop frame)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_image = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_visual', 10)
        self.pub_debug = self.create_publisher(Image, '/pcd_debug/image', qos)

        self.get_logger().info(
            'AMR Perception (PCD) node aktif — '
            'sub: /camera/camera/color/image_raw, pub: /cmd_vel_visual')

    # ---------------- pipeline PCD (identik dgn prototipe) ----------------
    def process(self, frame):
        h, w = frame.shape[:2]

        # 1-2. ROI crop + resize
        roi = frame[int(h * ROI_CROP_RATIO):h, 0:w]
        scale = TARGET_WIDTH / roi.shape[1]
        resized = cv2.resize(roi, (TARGET_WIDTH, int(roi.shape[0] * scale)),
                             interpolation=cv2.INTER_AREA)

        # 3. grayscale + blur
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)

        # 4. Otsu threshold
        _, mask = cv2.threshold(blurred, 0, 255,
                                cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 5. morfologi opening + closing
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 7. contour + area + posisi rata-rata obstacle
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        total_area = 0
        cx_sum, n_box = 0, 0
        debug = resized.copy()

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_AREA:
                total_area += area
                x, y, bw, bh = cv2.boundingRect(cnt)
                cx_sum += x + bw // 2
                n_box += 1
                cv2.rectangle(debug, (x, y), (x + bw, y + bh), (0, 0, 255), 2)

        obstacle_cx = (cx_sum / n_box) if n_box else TARGET_WIDTH / 2
        return total_area, obstacle_cx, debug

    # ---------------- callback utama ----------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge gagal: {e}')
            return

        total_area, obstacle_cx, debug = self.process(frame)

        # 8. logika zona -> Twist
        cmd = Twist()
        if total_area >= ZONE_DANGER:
            status, color = 'BAHAYA (STOP)', (0, 0, 255)
            cmd.linear.x = SPEED_STOP
        elif total_area >= ZONE_WARN:
            status, color = 'PERINGATAN (belok)', (0, 165, 255)
            cmd.linear.x = SPEED_WARN
            # obstacle di kiri frame -> belok kanan (z negatif), dan sebaliknya
            cmd.angular.z = STEER_WARN if obstacle_cx > TARGET_WIDTH / 2 \
                else -STEER_WARN
        else:
            status, color = 'AMAN (maju)', (0, 255, 0)
            cmd.linear.x = SPEED_SAFE

        self.pub_cmd.publish(cmd)

        # debug overlay
        cv2.putText(debug, f'AREA: {int(total_area)} | {status}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        try:
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AmrPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
