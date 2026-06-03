"""
collect_dataset.launch.py
==========================
Launch data_collector_node untuk record dataset visual regression.

Sebelum jalankan: pastikan
  1. ros2 launch amr_bringup amr_launch.py    (joystick + stm32_bridge)
  2. ros2 launch amr_bringup sensors_launch.py (LiDAR + RealSense)
  3. ros2 run amr_controller odometry_publisher.py (optional, untuk pose)
sudah running di terminal lain.

Usage:
    ros2 launch amr_visual_regression collect_dataset.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/home/azhar/datasets',
        description='Where to save dataset'
    )
    capture_rate_arg = DeclareLaunchArgument(
        'capture_rate', default_value='10.0',
        description='Capture rate Hz'
    )

    return LaunchDescription([
        output_dir_arg,
        capture_rate_arg,
        Node(
            package='amr_visual_regression',
            executable='data_collector_node',
            name='data_collector_node',
            output='screen',
            parameters=[{
                'output_dir':   LaunchConfiguration('output_dir'),
                'capture_rate': LaunchConfiguration('capture_rate'),
            }]
        ),
    ])
