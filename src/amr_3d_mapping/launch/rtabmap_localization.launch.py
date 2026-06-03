"""
rtabmap_localization.launch.py
===============================
Launch RTAB-Map dalam mode LOCALIZATION (untuk demo autonomous nav).

Pre-requisite: harus sudah ada file .db hasil mapping mode sebelumnya.
Mode ini READ-ONLY: tidak menambah node ke graph, hanya track pose di peta.

Usage:
  ros2 launch amr_3d_mapping rtabmap_localization.launch.py \\
      database_path:=~/maps/lab_3d.db
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/maps/lab_3d.db',
        description='Path ke .db hasil mapping mode (READ-ONLY)')

    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic', default_value='/camera/color/image_raw')
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/depth/image_rect_raw')
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/color/camera_info')
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom')

    config_path = PathJoinSubstitution([
        FindPackageShare('amr_3d_mapping'),
        'config',
        'rtabmap_localization.yaml'
    ])

    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[
            config_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('rgb/image',       LaunchConfiguration('rgb_topic')),
            ('depth/image',     LaunchConfiguration('depth_topic')),
            ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
        ],
    )

    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            config_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'database_path': LaunchConfiguration('database_path'),
            },
        ],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('scan',       LaunchConfiguration('scan_topic')),
            ('odom',       LaunchConfiguration('odom_topic')),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        scan_topic_arg,
        odom_topic_arg,
        rgbd_sync_node,
        rtabmap_slam_node,
    ])
