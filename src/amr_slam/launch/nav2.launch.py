"""
nav2.launch.py
===============
Launch Nav2 stack untuk Ackermann robot.

Penting:
  - cmd_vel topic Nav2 di-remap ke /cmd_vel_nav (bukan /cmd_vel default)
    agar bisa di-arbitrasi oleh failover_controller.

Usage:
    ros2 launch amr_slam nav2.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_slam')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(
        nav2_bringup_share, 'launch', 'navigation_launch.py'
    )

    # Remap /cmd_vel -> /cmd_vel_nav so failover_controller can arbitrate.
    nav2_with_remap = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time':    'false',
                'params_file':     nav2_params,
                'autostart':       'true',
            }.items(),
        ),
    ])

    return LaunchDescription([nav2_with_remap])
