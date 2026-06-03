"""
amr_full.launch.py
===================
MASTER LAUNCH FILE - Mobile Robot Ackermann Indoor Platform.

Komponen yang di-launch:
  1. URDF + robot_state_publisher (always)
  2. Joystick + stm32_bridge      (always)
  3. Sensor: LiDAR + RealSense    (always)
  4. Wheel odometry publisher     (always)
  5. SLAM Toolbox 2D               (default ON, mode 'mapping' atau 'localization')
  6. RTAB-Map 3D                   (default OFF, mode 'mapping' atau 'localization')
  7. Line Segments overlay         (default OFF, visual regression LiDAR)
  8. Nav2 stack                    (default OFF)
  9. Visual Regression inference   (default OFF, depth regression CNN-less)
 10. Failover controller           (default OFF)

Usage examples:
    # Foundation only (sensor + odometry + URDF, tanpa SLAM):
    ros2 launch amr_bringup amr_full.launch.py use_slam:=false

    # Mapping mode 2D + 3D simultan:
    ros2 launch amr_bringup amr_full.launch.py \\
        slam_mode:=mapping use_rtabmap:=true rtabmap_mode:=mapping

    # Line segments standalone (testing tanpa SLAM/Nav2):
    ros2 launch amr_bringup amr_full.launch.py \\
        use_slam:=false use_line_segments:=true

    # Full demo (localization 2D + 3D + Nav2 + line segments + Failover):
    ros2 launch amr_bringup amr_full.launch.py \\
        slam_mode:=localization use_rtabmap:=true rtabmap_mode:=localization \\
        use_nav2:=true use_line_segments:=true use_failover:=true \\
        map_name:=lab_map rtabmap_db_path:=~/maps/lab_3d.db
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_slam_arg = DeclareLaunchArgument(
        'use_slam', default_value='true',
        description='Enable SLAM Toolbox')
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode', default_value='mapping',
        description="'mapping' atau 'localization'")
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='lab_map',
        description='Nama map (tanpa extension)')
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='false',
        description='Enable Nav2 stack')
    use_vr_arg = DeclareLaunchArgument(
        'use_vr', default_value='false',
        description='Enable Visual Regression inference')
    use_failover_arg = DeclareLaunchArgument(
        'use_failover', default_value='false',
        description='Enable failover controller')
    vr_model_path_arg = DeclareLaunchArgument(
        'vr_model_path', default_value='/home/azhar/models/vr_model.pkl')
    vr_scaler_path_arg = DeclareLaunchArgument(
        'vr_scaler_path', default_value='/home/azhar/models/vr_scaler.pkl')

    # ---- RTAB-Map 3D (new) ----
    use_rtabmap_arg = DeclareLaunchArgument(
        'use_rtabmap', default_value='false',
        description='Enable RTAB-Map 3D mapping/localization (RGB-D + LiDAR sync)')
    rtabmap_mode_arg = DeclareLaunchArgument(
        'rtabmap_mode', default_value='mapping',
        description="'mapping' (build new 3D map) atau 'localization' (load existing .db)")
    rtabmap_db_path_arg = DeclareLaunchArgument(
        'rtabmap_db_path', default_value='~/.ros/rtabmap.db',
        description='Path ke RTAB-Map database .db')

    # ---- Line Segments overlay (new) ----
    use_line_segments_arg = DeclareLaunchArgument(
        'use_line_segments', default_value='false',
        description='Enable LiDAR line segments RANSAC overlay (visual regression)')

    use_slam = LaunchConfiguration('use_slam')
    slam_mode = LaunchConfiguration('slam_mode')
    map_name = LaunchConfiguration('map_name')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_vr = LaunchConfiguration('use_vr')
    use_failover = LaunchConfiguration('use_failover')
    use_rtabmap = LaunchConfiguration('use_rtabmap')
    rtabmap_mode = LaunchConfiguration('rtabmap_mode')
    rtabmap_db_path = LaunchConfiguration('rtabmap_db_path')
    use_line_segments = LaunchConfiguration('use_line_segments')

    pkg_bringup = FindPackageShare('amr_bringup')
    pkg_desc = FindPackageShare('amr_description')
    pkg_slam = FindPackageShare('amr_slam')
    pkg_vr = FindPackageShare('amr_visual_regression')
    pkg_fo = FindPackageShare('amr_failover')
    pkg_3d = FindPackageShare('amr_3d_mapping')

    urdf_xacro = PathJoinSubstitution(
        [pkg_desc, 'urdf', 'amr_description.urdf.xacro'])
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)}],
    )

    bringup_amr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'amr_launch.py'])
        )
    )

    bringup_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'sensors_launch.py'])
        )
    )

    odom_node = Node(
        package='amr_controller',
        executable='odometry_publisher.py',
        name='odometry_publisher',
        output='screen',
        parameters=[{
            'wheel_radius':         0.0775,
            'wheelbase':            0.50,
            'pulses_per_revolution': 1496,
            'encoder_period':       0.05,
            'publish_rate':         50.0,
            'publish_tf':           True,
        }],
    )

    slam_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, 'launch', 'slam_mapping.launch.py'])
        ),
        condition=IfCondition(
            PythonExpression(
                ["'", use_slam, "' == 'true' and '", slam_mode, "' == 'mapping'"])
        ),
    )

    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, 'launch', 'slam_localization.launch.py'])
        ),
        launch_arguments={'map_name': map_name}.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", use_slam, "' == 'true' and '", slam_mode, "' == 'localization'"])
        ),
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, 'launch', 'nav2.launch.py'])
        ),
        condition=IfCondition(use_nav2),
    )

    vr_inference = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_vr, 'launch', 'vr_inference.launch.py'])
        ),
        launch_arguments={
            'model_path':  LaunchConfiguration('vr_model_path'),
            'scaler_path': LaunchConfiguration('vr_scaler_path'),
        }.items(),
        condition=IfCondition(use_vr),
    )

    failover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_fo, 'launch', 'failover.launch.py'])
        ),
        condition=IfCondition(use_failover),
    )

    # =================================================================
    # NEW: RTAB-Map 3D mapping (sync mode RGB-D + LiDAR)
    # =================================================================
    rtabmap_mapping_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_3d, 'launch', 'rtabmap_mapping.launch.py'])
        ),
        launch_arguments={
            'database_path': rtabmap_db_path,
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", use_rtabmap, "' == 'true' and '", rtabmap_mode, "' == 'mapping'"])
        ),
    )

    rtabmap_localization_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_3d, 'launch', 'rtabmap_localization.launch.py'])
        ),
        launch_arguments={
            'database_path': rtabmap_db_path,
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", use_rtabmap, "' == 'true' and '", rtabmap_mode, "' == 'localization'"])
        ),
    )

    # =================================================================
    # NEW: LiDAR line segments overlay (visual regression)
    # =================================================================
    line_segments_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_vr, 'launch', 'line_segments.launch.py'])
        ),
        condition=IfCondition(use_line_segments),
    )

    return LaunchDescription([
        # Original launch args
        use_slam_arg, slam_mode_arg, map_name_arg,
        use_nav2_arg, use_vr_arg, use_failover_arg,
        vr_model_path_arg, vr_scaler_path_arg,
        # New launch args (RTAB-Map + line segments)
        use_rtabmap_arg, rtabmap_mode_arg, rtabmap_db_path_arg,
        use_line_segments_arg,
        # Original nodes/includes
        rsp_node,
        bringup_amr,
        bringup_sensors,
        odom_node,
        slam_mapping,
        slam_localization,
        nav2_stack,
        vr_inference,
        failover,
        # New includes
        rtabmap_mapping_inc,
        rtabmap_localization_inc,
        line_segments_inc,
    ])
