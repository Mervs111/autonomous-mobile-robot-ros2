"""
view_robot.launch.py
=====================
Visualisasi URDF AMR di RViz dengan joint_state_publisher_gui untuk
menggeser sudut steering manual.

Usage:
    ros2 launch amr_description view_robot.launch.py
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('amr_description')

    urdf_file = PathJoinSubstitution(
        [pkg_share, 'urdf', 'amr_description.urdf.xacro']
    )
    rviz_config = PathJoinSubstitution(
        [pkg_share, 'rviz', 'view_robot.rviz']
    )

    robot_description = {'robot_description': Command(['xacro ', urdf_file])}

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # GUI slider untuk menggeser steering joints (testing URDF)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

    ])
