"""
slam_mapping.launch.py
=======================
Launch SLAM Toolbox dalam mode online async mapping.

Pre-requisite (jalankan di terminal lain):
    1. ros2 launch amr_bringup amr_launch.py        (joystick + stm32_bridge)
    2. ros2 launch amr_bringup sensors_launch.py    (LiDAR + camera)
    3. ros2 run amr_controller odometry_publisher.py (odometry + TF odom->base_link)
    4. ros2 launch amr_description view_robot.launch.py  (URDF + robot_state_publisher)
       (atau gabungkan via amr_full.launch.py)

Setelah selesai mapping:
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
        "{name: {data: 'lab_map'}}"

Usage:
    ros2 launch amr_slam slam_mapping.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_slam')
    config = os.path.join(pkg_share, 'config', 'slam_mapping.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config, {'use_sim_time': False}],
        ),
    ])
