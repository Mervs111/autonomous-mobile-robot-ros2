"""
slam_localization.launch.py
============================
Launch SLAM Toolbox dalam mode localization (load map yang sudah disimpan).

Usage:
    ros2 launch amr_slam slam_localization.launch.py map_name:=lab_map
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_slam')
    config = os.path.join(pkg_share, 'config', 'slam_localization.yaml')

    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='lab_map',
        description='Nama file map (tanpa extension) di share/amr_slam/maps/'
    )
    map_file = PathJoinSubstitution([
        FindPackageShare('amr_slam'), 'maps',
        LaunchConfiguration('map_name')
    ])

    return LaunchDescription([
        map_name_arg,
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                config,
                {'use_sim_time': False, 'map_file_name': map_file}
            ],
        ),
    ])
