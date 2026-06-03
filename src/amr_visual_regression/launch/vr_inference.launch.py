"""
vr_inference.launch.py
=======================
Launch vr_inference_node untuk real-time inference Visual Regression.

Pastikan model & scaler sudah ada (hasil dari scripts/train.py).

Usage:
    ros2 launch amr_visual_regression vr_inference.launch.py \\
        model_path:=/home/azhar/models/vr_model.pkl \\
        scaler_path:=/home/azhar/models/vr_scaler.pkl
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path',
            default_value='/home/azhar/models/vr_model.pkl'),
        DeclareLaunchArgument('scaler_path',
            default_value='/home/azhar/models/vr_scaler.pkl'),
        DeclareLaunchArgument('cmd_topic',
            default_value='/cmd_vel_visual'),

        Node(
            package='amr_visual_regression',
            executable='vr_inference_node',
            name='vr_inference_node',
            output='screen',
            parameters=[{
                'model_path':    LaunchConfiguration('model_path'),
                'scaler_path':   LaunchConfiguration('scaler_path'),
                'cmd_topic':     LaunchConfiguration('cmd_topic'),
                'num_regions':   9,
                'roi_top':       200,
                'roi_bottom':    360,
                'safety_min_depth': 0.4,
                'vx_max':        0.4,
                'steer_max_rad': 0.785,
                'publish_rate':  10.0,
            }],
        ),
    ])
