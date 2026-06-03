"""
failover.launch.py
===================
Launch failover_controller node.

Usage:
    ros2 launch amr_failover failover.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_failover',
            executable='failover_controller',
            name='failover_controller',
            output='screen',
            parameters=[{
                'control_rate':         20.0,
                'fallback_delay_s':      2.0,
                'recovery_delay_s':      5.0,
                'scan_timeout_s':        1.0,
                'map_timeout_s':        10.0,
                'visual_timeout_s':      0.5,
                'emergency_min_range':   0.30,
                'deadman_button':        5,
                'cmd_topic':             '/cmd_vel',
                'cmd_nav_topic':         '/cmd_vel_nav',
                'cmd_visual_topic':      '/cmd_vel_visual',
                'cmd_joy_topic':         '/cmd_vel_joy',
            }],
        ),
    ])
