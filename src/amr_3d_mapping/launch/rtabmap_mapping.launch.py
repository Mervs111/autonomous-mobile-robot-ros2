"""
rtabmap_mapping.launch.py  (VIO revision — 26 Mei 2026)
==========================
Launch RTAB-Map dalam mode MAPPING dengan Visual-Inertial Odometry (VIO).

PERUBAHAN dari versi sebelumnya:
  - TAMBAH: imu_merger_node (merge accel + gyro → /imu/data)
  - TAMBAH: rgbd_odometry node (VIO: RGB-D + IMU → /rtabmap/odom)
  - FIX: default topic names ke /camera/camera/... (double namespace D455)
  - FIX: rtabmap SLAM subscribe ke /rtabmap/odom (bukan wheel /odom)
  - HAPUS: dependensi ke /odom wheel odometry sebagai primary odometry

Wheel odometry (odometry_publisher.py) tetap jalan tapi:
  - Tidak dipakai sebagai input RTAB-Map
  - Kalau publish_tf masih true di odometry_publisher → TF conflict
  - Jalankan patch_odom_publisher_no_tf.py sebelum deploy ini

Prerequisite (dari amr_full.launch.py):
  - realsense2_camera_node publishing:
      /camera/camera/color/image_raw
      /camera/camera/depth/image_rect_raw
      /camera/camera/color/camera_info
      /camera/camera/accel/sample  (100 Hz)
      /camera/camera/gyro/sample   (200 Hz)
  - rplidar_node publishing: /scan
  - robot_state_publisher publishing TF base_link → camera_link, laser_frame
  - static_tf bridges: color_optical_frame <-> camera_color_optical_frame

Output:
  - /rtabmap/odom           : VIO pose (dari rgbd_odometry)
  - /rtabmap/cloud_map      : peta 3D point cloud
  - /rtabmap/grid_map       : occupancy grid 2D (untuk Nav2)
  - TF map → odom → base_link (chain lengkap)

Usage:
  # Standalone:
  ros2 launch amr_3d_mapping rtabmap_mapping.launch.py

  # Dengan database path custom (fresh mapping):
  ros2 launch amr_3d_mapping rtabmap_mapping.launch.py \
      database_path:=~/maps/lab_vio.db
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---- Launch arguments ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time (gazebo only)')

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value='~/.ros/rtabmap.db',
        description='Path ke file .db RTAB-Map')

    # delete_db_on_start: tidak di-wire sebagai arg karena rtabmap ROS 2
    # butuh conditional logic yang kompleks. Untuk fresh mapping, delete manual:
    #   rm ~/.ros/rtabmap.db
    # ATAU ganti database_path ke file baru:
    #   database_path:=~/maps/lab_vio_NEW.db

    # ---- Topic args: D455 double namespace (/camera/camera/...) ----
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/camera/color/image_raw',
        description='RGB image topic (D455: /camera/camera/color/image_raw)')

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/aligned_depth_to_color/image_raw',
        description='Depth aligned ke RGB (dari align_depth.enable=True)')

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='RGB camera info')

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='LiDAR scan topic')

    # ---- Config path ----
    config_path = PathJoinSubstitution([
        FindPackageShare('amr_3d_mapping'),
        'config',
        'rtabmap_mapping.yaml'
    ])

    # =================================================================
    # NODE 1: IMU MERGER
    # Gabungkan /camera/camera/accel/sample (100 Hz) dan
    # /camera/camera/gyro/sample (200 Hz) → /imu/data (Imu message)
    # untuk dikonsumsi rgbd_odometry.
    # File: ~/amr_starter/src/amr_controller/scripts/imu_merger_node.py
    # =================================================================
    imu_merger_node = Node(
        package='amr_controller',
        executable='imu_merger_node.py',
        name='imu_merger',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'accel_topic': '/camera/camera/accel/sample',
                'gyro_topic': '/camera/camera/gyro/sample',
                # output_frame_id: '' (default) = preserve gyro's frame_id
                # = camera_gyro_optical_frame, terhubung ke base_link via RealSense TF
            }
        ],
    )

    # =================================================================
    # NODE 2: RGB-D SYNC (rgbd_sync)
    # Sync RGB + Depth + CameraInfo → satu /rgbd_image dengan timestamp aligned.
    # =================================================================
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

    # =================================================================
    # NODE 3: RGB-D ODOMETRY (rgbd_odometry)
    # Visual-Inertial Odometry: RGB-D features + IMU → pose tracking
    # Menggantikan wheel odometry sebagai sumber /rtabmap/odom
    # dan TF odom → base_link.
    # =================================================================
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[
            config_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_rgbd': True,
                'subscribe_imu': True,
                'approx_sync': True,
                'queue_size': 30,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,     # publish TF odom → base_link
            }
        ],
        remappings=[
            ('rgbd_image', '/rgbd_image'),   # dari rgbd_sync
            ('imu',        '/imu/data'),      # dari imu_merger
            ('odom',       '/rtabmap/odom'),  # output VIO → /rtabmap/odom
        ],
    )

    # =================================================================
    # NODE 4: RTAB-Map SLAM (rtabmap)
    # Engine utama: graph-based SLAM, loop closure, map optimization.
    # Konsumsi VIO dari rgbd_odometry (bukan wheel odom).
    # =================================================================
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
                # ---- Subscriptions (inline = pasti dimuat) ----
                'subscribe_depth': False,
                'subscribe_rgbd': True,
                'subscribe_scan': True,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'queue_size': 30,
                'sync_queue_size': 30,
                # ---- Frames ----
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                # ---- TF ----
                'tf_delay': 0.05,
                'tf_tolerance': 0.1,
                'wait_for_transform': 0.2,
                'publish_tf': True,
                # ---- Database ----
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Mem/SaveDepth16Format': 'true',
                'Mem/DepthAsMask': 'false',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/STMSize': '30',
                'Mem/UseOdomFeatures': 'false',
                # ---- Registration: Vis+ICP ----
                'Reg/Strategy': '2',
                'Reg/Force3DoF': 'true',
                'Icp/PointToPlane': 'true',
                'Icp/Iterations': '15',
                'Icp/VoxelSize': '0.05',
                'Icp/MaxCorrespondenceDistance': '0.1',
                'Icp/Epsilon': '0.001',
                'Icp/MaxTranslation': '1.0',
                'Icp/MaxRotation': '0.78',
                # ---- Visual features ----
                'Vis/FeatureType': '8',
                'Vis/MaxFeatures': '1000',
                'Vis/MinInliers': '10',
                # ---- Grid / occupancy map ----
                'Grid/FromDepth': 'false',
                'Grid/RangeMax': '5.0',
                'Grid/CellSize': '0.05',
                'Grid/RayTracing': 'true',
                'Grid/3D': 'false',
                'Grid/NormalsSegmentation': 'false',
                'Grid/MaxObstacleHeight': '1.5',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MinGroundHeight': '-0.05',
                # ---- Loop closure ----
                'Rtabmap/LoopThr': '0.11',
                'Rtabmap/DetectionRate': '1.0',
                'Rtabmap/TimeThr': '700',
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/LocalRadius': '5.0',
                'RGBD/ProximityMaxGraphDepth': '50',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/ProximityByTime': 'false',
                # ---- Optimizer ----
                'Optimizer/GravitySigma': '0.3',
                # ---- Publishing ----
                'publish_cloud_map': False,
                'publish_cloud_obstacles': True,
                'publish_cloud_ground': True,
                'publish_grid_map': True,
            },
        ],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('scan',       LaunchConfiguration('scan_topic')),
            ('odom',       '/rtabmap/odom'),
        ],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # =================================================================
    # NODE 5: depthimage_to_laserscan
    # Konversi depth image D455 → virtual LaserScan untuk Nav2 costmap.
    # Mendeteksi obstacle yang tidak terlihat LiDAR (terlalu rendah/tinggi).
    # Output: /depth_scan (LaserScan) → dikonsumsi Nav2 local + global costmap.
    # =================================================================
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_height': 10,           # jumlah baris pixel yang di-sample
            'scan_time': 0.033,          # 30Hz sesuai RealSense
            'range_min': 0.2,            # jarak minimum valid (meter)
            'range_max': 5.0,            # jarak maximum valid (meter)
            'output_frame': 'camera_color_optical_frame',
        }],
        remappings=[
            ('image',       LaunchConfiguration('depth_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
            ('scan',        '/depth_scan'),  # output terpisah dari /scan LiDAR
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        database_path_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        scan_topic_arg,
        imu_merger_node,       # 1. IMU merger dulu
        rgbd_sync_node,        # 2. Sync RGB-D
        rgbd_odometry_node,    # 3. VIO odometry
        rtabmap_slam_node,     # 4. SLAM engine
        depth_to_scan_node,    # 5. Depth → virtual scan untuk Nav2 costmap
    ])
