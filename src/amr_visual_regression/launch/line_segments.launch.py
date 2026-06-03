"""
line_segments.launch.py
========================
Launch standalone untuk lidar_line_segments_node.

Use case:
  - Testing cepat tanpa full system: tinggal pastikan /scan publishing
    (dari rplidar atau dari rosbag playback), lalu jalankan ini.
  - Tuning parameter RANSAC interaktif via ros2 param set.

Usage:
  ros2 launch amr_visual_regression line_segments.launch.py
  ros2 launch amr_visual_regression line_segments.launch.py \\
      ransac_iterations:=50 inlier_threshold_m:=0.03

Visualisasi di Foxglove:
  - Topic /amr/line_segments di panel 3D, dengan fixed_frame=laser
  - Topic /amr/line_count di panel Plot untuk monitor count over time
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan')
    max_range_arg = DeclareLaunchArgument(
        'max_range_m', default_value='8.0')
    gap_threshold_arg = DeclareLaunchArgument(
        'gap_threshold_m', default_value='0.30')
    ransac_iter_arg = DeclareLaunchArgument(
        'ransac_iterations', default_value='30')
    inlier_thr_arg = DeclareLaunchArgument(
        'inlier_threshold_m', default_value='0.05')
    min_inliers_arg = DeclareLaunchArgument(
        'min_inliers', default_value='8')
    max_lines_arg = DeclareLaunchArgument(
        'max_lines_per_cluster', default_value='3')
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz', default_value='5.0')
    line_width_arg = DeclareLaunchArgument(
        'line_width', default_value='0.03')

    line_segments_node = Node(
        package='amr_visual_regression',
        executable='lidar_line_segments_node',
        name='lidar_line_segments_node',
        output='screen',
        parameters=[{
            'scan_topic':            LaunchConfiguration('scan_topic'),
            'max_range_m':           LaunchConfiguration('max_range_m'),
            'gap_threshold_m':       LaunchConfiguration('gap_threshold_m'),
            'ransac_iterations':     LaunchConfiguration('ransac_iterations'),
            'inlier_threshold_m':    LaunchConfiguration('inlier_threshold_m'),
            'min_inliers':           LaunchConfiguration('min_inliers'),
            'max_lines_per_cluster': LaunchConfiguration('max_lines_per_cluster'),
            'publish_rate_hz':       LaunchConfiguration('publish_rate_hz'),
            'line_width':            LaunchConfiguration('line_width'),
        }],
    )

    return LaunchDescription([
        scan_topic_arg,
        max_range_arg,
        gap_threshold_arg,
        ransac_iter_arg,
        inlier_thr_arg,
        min_inliers_arg,
        max_lines_arg,
        publish_rate_arg,
        line_width_arg,
        line_segments_node,
    ])
