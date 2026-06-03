#!/usr/bin/env python3
"""
imu_merger_node.py
==================
Merge accel-only and gyro-only IMU streams from Intel RealSense D455
into single complete IMU message for robot_localization EKF.

RATIONALE:
  RealSense D455 publishes accel and gyro as separate topics, each as
  sensor_msgs/Imu but with only partial fields filled:
    - /camera/camera/accel/sample @ ~100 Hz (linear_acceleration only)
    - /camera/camera/gyro/sample  @ ~200 Hz (angular_velocity only)
  
  robot_localization EKF expects a single sensor_msgs/Imu with both
  linear_acceleration AND angular_velocity in one message.

APPROACH:
  - message_filters.ApproximateTimeSynchronizer pairs by timestamp (slop 50ms)
  - Output published at the slower input rate (~100 Hz)
  - Orientation = identity quaternion (no AHRS for emergency deployment)
  - orientation_covariance[0] = -1.0 signals to robot_localization
    "orientation not available, ignore it"

FRAME_ID:
  Output uses the gyro message's frame_id by default (since gyro is more
  critical for vyaw fusion). Common RealSense conventions:
    - camera_gyro_optical_frame (most common)
    - camera_imu_optical_frame (some URDFs use this)
  Configurable via parameter 'output_frame_id'. If you leave it empty,
  the gyro's own frame_id is preserved.

USAGE:
  # Standalone:
  python3 imu_merger_node.py
  
  # Via launch (see ekf_emergency.launch.py):
  ros2 launch amr_bringup ekf_emergency.launch.py

Author: Claude (sesi 20 Mei 2026)
Untuk: AMR Ackermann ITS — EKF emergency standby package
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer


# Default covariance values for RealSense D455 IMU (BMI055 chip)
# Reference: BMI055 datasheet
#   - Accel noise density: ~150 µg/√Hz ⇒ at 100 Hz ⇒ ~1.5 mg RMS ≈ 0.015 m/s²
#   - Gyro noise density: ~0.014 °/s/√Hz ⇒ at 200 Hz ⇒ ~0.2 °/s ≈ 0.0035 rad/s
# These are conservative (slightly higher than spec) for emergency deployment.
DEFAULT_ACCEL_COVARIANCE = 0.01    # (m/s²)² diagonal
DEFAULT_GYRO_COVARIANCE = 0.001    # (rad/s)² diagonal


class ImuMerger(Node):
    def __init__(self):
        super().__init__('imu_merger')
        
        # === Parameters ===
        self.declare_parameter('accel_topic', '/camera/camera/accel/sample')
        self.declare_parameter('gyro_topic', '/camera/camera/gyro/sample')
        self.declare_parameter('output_topic', '/imu/data')
        # Empty string = preserve gyro's frame_id. Set explicitly to override.
        self.declare_parameter('output_frame_id', '')
        self.declare_parameter('sync_slop_seconds', 0.05)
        self.declare_parameter('linear_acceleration_covariance', DEFAULT_ACCEL_COVARIANCE)
        self.declare_parameter('angular_velocity_covariance', DEFAULT_GYRO_COVARIANCE)
        
        accel_topic = self.get_parameter('accel_topic').value
        gyro_topic = self.get_parameter('gyro_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        slop = float(self.get_parameter('sync_slop_seconds').value)
        self.accel_cov = float(self.get_parameter('linear_acceleration_covariance').value)
        self.gyro_cov = float(self.get_parameter('angular_velocity_covariance').value)
        
        # === Publisher ===
        # Use BEST_EFFORT QoS to match RealSense sensor stream default
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.imu_pub = self.create_publisher(Imu, output_topic, sensor_qos)
        
        # === message_filters subscribers ===
        # NOTE: message_filters in rclpy needs the QoS as keyword arg
        self.accel_sub = Subscriber(self, Imu, accel_topic, qos_profile=sensor_qos)
        self.gyro_sub = Subscriber(self, Imu, gyro_topic, qos_profile=sensor_qos)
        
        self.sync = ApproximateTimeSynchronizer(
            [self.accel_sub, self.gyro_sub],
            queue_size=20,
            slop=slop,
        )
        self.sync.registerCallback(self.imu_callback)
        
        # === Stats logging ===
        self.msg_count = 0
        self.create_timer(5.0, self._log_stats)
        
        self.get_logger().info(
            f'IMU merger initialized:\n'
            f'  accel input  : {accel_topic}\n'
            f'  gyro  input  : {gyro_topic}\n'
            f'  output topic : {output_topic}\n'
            f'  output frame : '
            f'{self.output_frame_id if self.output_frame_id else "(preserved from gyro)"}\n'
            f'  sync slop    : {slop:.3f} s\n'
            f'  accel cov    : {self.accel_cov} (m/s²)²\n'
            f'  gyro  cov    : {self.gyro_cov} (rad/s)²'
        )
    
    def _log_stats(self):
        rate = self.msg_count / 5.0
        if rate < 50.0:
            self.get_logger().warn(
                f'Merged IMU rate LOW: {rate:.1f} Hz (expected ~100 Hz). '
                f'Check accel/gyro topics publishing & sync slop tolerance.'
            )
        else:
            self.get_logger().info(f'Merged IMU rate: {rate:.1f} Hz (last 5s)')
        self.msg_count = 0
    
    def imu_callback(self, accel_msg: Imu, gyro_msg: Imu):
        """Called when accel+gyro pair within slop tolerance arrive."""
        merged = Imu()
        
        # Use gyro timestamp (higher rate ⇒ more recent for vyaw)
        merged.header.stamp = gyro_msg.header.stamp
        # Frame: parameter override > gyro's own frame_id
        if self.output_frame_id:
            merged.header.frame_id = self.output_frame_id
        else:
            merged.header.frame_id = gyro_msg.header.frame_id
        
        # === Orientation: identity, marked as "not available" ===
        # robot_localization checks orientation_covariance[0] == -1.0
        # to skip orientation usage. This is the canonical signal.
        merged.orientation.x = 0.0
        merged.orientation.y = 0.0
        merged.orientation.z = 0.0
        merged.orientation.w = 1.0
        merged.orientation_covariance = [-1.0] + [0.0] * 8
        
        # === Linear acceleration from accel msg ===
        merged.linear_acceleration = accel_msg.linear_acceleration
        merged.linear_acceleration_covariance = [
            self.accel_cov, 0.0, 0.0,
            0.0, self.accel_cov, 0.0,
            0.0, 0.0, self.accel_cov,
        ]
        
        # === Angular velocity from gyro msg ===
        merged.angular_velocity = gyro_msg.angular_velocity
        merged.angular_velocity_covariance = [
            self.gyro_cov, 0.0, 0.0,
            0.0, self.gyro_cov, 0.0,
            0.0, 0.0, self.gyro_cov,
        ]
        
        self.imu_pub.publish(merged)
        self.msg_count += 1


def main():
    rclpy.init()
    node = ImuMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
