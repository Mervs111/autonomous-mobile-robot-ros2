#!/usr/bin/env python3
"""
failover_controller.py
=======================
Arbiter cmd_vel untuk Mobile Robot Ackermann Indoor Platform.

State machine:
    SLAM_ACTIVE      (default)  -> publish /cmd_vel = /cmd_vel_nav
    VISUAL_FALLBACK             -> publish /cmd_vel = /cmd_vel_visual
    JOY_OVERRIDE                -> publish /cmd_vel = /cmd_vel_joy
    EMERGENCY_STOP              -> publish /cmd_vel = (0, 0)

Triggers:
    SLAM_ACTIVE -> VISUAL_FALLBACK : SLAM unhealthy selama fallback_delay_s
    VISUAL_FALLBACK -> SLAM_ACTIVE : SLAM healthy selama recovery_delay_s (hysteresis)
    Any -> JOY_OVERRIDE            : Deadman R1 ditekan
    Any -> EMERGENCY_STOP          : min /scan range < emergency_min_range
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker


ST_SLAM = 'SLAM_ACTIVE'
ST_VISUAL = 'VISUAL_FALLBACK'
ST_JOY = 'JOY_OVERRIDE'
ST_ESTOP = 'EMERGENCY_STOP'

STATE_COLORS = {
    ST_SLAM:   (0.0, 1.0, 0.0),
    ST_VISUAL: (1.0, 1.0, 0.0),
    ST_JOY:    (0.0, 0.5, 1.0),
    ST_ESTOP:  (1.0, 0.0, 0.0),
}


class FailoverController(Node):

    def __init__(self):
        super().__init__('failover_controller')

        # Parameters
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('fallback_delay_s', 2.0)
        self.declare_parameter('recovery_delay_s', 5.0)
        self.declare_parameter('scan_timeout_s', 1.0)
        self.declare_parameter('map_timeout_s',  10.0)
        self.declare_parameter('visual_timeout_s', 0.5)
        self.declare_parameter('emergency_min_range', 0.30)
        self.declare_parameter('deadman_button', 5)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('cmd_nav_topic',    '/cmd_vel_nav')
        self.declare_parameter('cmd_visual_topic', '/cmd_vel_visual')
        self.declare_parameter('cmd_joy_topic',    '/cmd_vel_joy')

        gp = lambda n: self.get_parameter(n).value
        self.fallback_delay = gp('fallback_delay_s')
        self.recovery_delay = gp('recovery_delay_s')
        self.scan_timeout = gp('scan_timeout_s')
        self.map_timeout = gp('map_timeout_s')
        self.visual_timeout = gp('visual_timeout_s')
        self.emergency_min_range = gp('emergency_min_range')
        self.deadman_btn = gp('deadman_button')

        # State
        self.state = ST_SLAM
        self.unhealthy_since = None
        self.healthy_since = None

        self.last_scan_t = None
        self.last_map_t = None
        self.last_visual_t = None
        self.last_min_scan = float('inf')
        self.deadman = False

        self.cmd_nav = Twist()
        self.cmd_visual = Twist()
        self.cmd_joy = Twist()

        # ROS interface
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.create_subscription(LaserScan, '/scan',
                                 self.cb_scan, qos_be)
        self.create_subscription(OccupancyGrid, '/map',
                                 self.cb_map, qos)
        self.create_subscription(Twist, gp('cmd_nav_topic'),
                                 self.cb_cmd_nav, qos)
        self.create_subscription(Twist, gp('cmd_visual_topic'),
                                 self.cb_cmd_visual, qos)
        self.create_subscription(Twist, gp('cmd_joy_topic'),
                                 self.cb_cmd_joy, qos)
        self.create_subscription(Joy, '/joy', self.cb_joy, qos)

        self.pub_cmd = self.create_publisher(Twist, gp('cmd_topic'), 10)
        self.pub_status = self.create_publisher(String, '/failover_status', 10)
        self.pub_marker = self.create_publisher(Marker, '/failover_marker', 10)

        period = 1.0 / max(gp('control_rate'), 1.0)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info('Failover controller started in SLAM_ACTIVE state.')

    def cb_scan(self, msg: LaserScan):
        self.last_scan_t = time.time()
        valid = [r for r in msg.ranges
                 if not math.isnan(r) and not math.isinf(r) and r > 0.05]
        self.last_min_scan = min(valid) if valid else float('inf')

    def cb_map(self, msg: OccupancyGrid):
        self.last_map_t = time.time()

    def cb_cmd_nav(self, msg: Twist):
        self.cmd_nav = msg

    def cb_cmd_visual(self, msg: Twist):
        self.cmd_visual = msg
        self.last_visual_t = time.time()

    def cb_cmd_joy(self, msg: Twist):
        self.cmd_joy = msg

    def cb_joy(self, msg: Joy):
        if len(msg.buttons) > self.deadman_btn:
            self.deadman = (msg.buttons[self.deadman_btn] == 1)

    def tick(self):
        now = time.time()

        scan_age = (now - self.last_scan_t) if self.last_scan_t else float('inf')
        map_age = (now - self.last_map_t) if self.last_map_t else float('inf')
        visual_age = (now - self.last_visual_t) if self.last_visual_t else float('inf')

        slam_healthy = (scan_age < self.scan_timeout) and (map_age < self.map_timeout)
        visual_healthy = (visual_age < self.visual_timeout)

        new_state = self.state

        # 1. Emergency stop (highest priority)
        if self.last_min_scan < self.emergency_min_range:
            new_state = ST_ESTOP
        # 2. Joy override
        elif self.deadman:
            new_state = ST_JOY
        # 3. Health-based with hysteresis
        elif self.state in (ST_SLAM, ST_ESTOP):
            if not slam_healthy:
                if self.unhealthy_since is None:
                    self.unhealthy_since = now
                elif (now - self.unhealthy_since) >= self.fallback_delay:
                    if visual_healthy:
                        new_state = ST_VISUAL
            else:
                self.unhealthy_since = None
                new_state = ST_SLAM
        elif self.state == ST_VISUAL:
            if slam_healthy:
                if self.healthy_since is None:
                    self.healthy_since = now
                elif (now - self.healthy_since) >= self.recovery_delay:
                    new_state = ST_SLAM
            else:
                self.healthy_since = None
        elif self.state == ST_JOY:
            if not self.deadman:
                new_state = ST_SLAM if slam_healthy else ST_VISUAL

        if new_state != self.state:
            self.get_logger().info(f'STATE CHANGE: {self.state} -> {new_state}')
            self.state = new_state
            if new_state == ST_SLAM:
                self.unhealthy_since = None
            if new_state == ST_VISUAL:
                self.healthy_since = None

        # Output cmd_vel
        if self.state == ST_SLAM:
            out = self.cmd_nav
        elif self.state == ST_VISUAL:
            out = self.cmd_visual
        elif self.state == ST_JOY:
            out = self.cmd_joy
        else:
            out = Twist()
        self.pub_cmd.publish(out)

        # Status
        status = {
            'state': self.state,
            'slam_healthy': slam_healthy,
            'visual_healthy': visual_healthy,
            'scan_age_s': round(scan_age, 3),
            'map_age_s': round(map_age, 3),
            'visual_age_s': round(visual_age, 3),
            'min_scan_range_m': round(float(self.last_min_scan), 3),
            'deadman': self.deadman,
        }
        s = String()
        s.data = json.dumps(status)
        self.pub_status.publish(s)

        # Marker
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'failover'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.7
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        rgb = STATE_COLORS.get(self.state, (1.0, 1.0, 1.0))
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 0.9
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = FailoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
