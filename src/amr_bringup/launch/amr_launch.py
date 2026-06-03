import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        # Node 1: Baca joystick → /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        ),

        # Node 2: /joy → Serial STM32
        # (deadman switch & konversi ditangani di dalam bridge)
        Node(
            package='amr_controller',
            executable='stm32_bridge',
            name='stm32_bridge',
            output='screen'
        ),

    ])
