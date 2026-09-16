"""Rover localization stack (REP-105 compliant Dual-EKF architecture).

Components:
1. Local EKF (ekf_filter_node_odom):
   Fuses wheel odometry velocities + IMU orientation/angular velocities.
   Publishes continuous, jump-free transform: odom -> base_footprint.
   Used by local costmaps and velocity controllers.

2. Global EKF (ekf_filter_node_map):
   Fuses wheel odometry velocities + IMU + absolute GPS fixes (/odometry/gps).
   Publishes global transform: map -> odom.
   Used by global path planners and waypoint followers.

3. Navsat Transform (navsat_transform):
   Transforms raw GPS fixes (/gps/fix) + IMU heading + Gazebo datum into
   Cartesian map-frame positions (/odometry/gps).

4. Ground Truth Node (ground_truth_node):
   Bridges true simulation pose (/gazebo/dynamic_pose) to /ground_truth/pose
   and broadcasts map -> ground_truth_base for real-time benchmarking and RViz.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_localization = get_package_share_directory('rover_localization')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    ekf_config_path = os.path.join(pkg_rover_localization, 'config', 'ekf.yaml')
    fast_lio_config_path = os.path.join(pkg_fast_lio, 'config', 'rover.yaml')

    use_fast_lio_arg = DeclareLaunchArgument(
        'use_fast_lio',
        default_value='false',
        description='Whether to use Fast-LIO2 (true) or KISS-ICP (false) for 3D LiDAR odometry'
    )
    use_fast_lio = LaunchConfiguration('use_fast_lio')

    use_oracle_arg = DeclareLaunchArgument(
        'use_oracle',
        default_value='true',
        description='Whether to use low-rate 3D satellite oracle and global 3D corrector'
    )
    use_oracle = LaunchConfiguration('use_oracle')

    initial_z_arg = DeclareLaunchArgument(
        'initial_z',
        default_value='1.594',
        description='Initial ground elevation datum for map->odom Z offset (1.594 for Marsyard, 0.0 for calibration world)'
    )
    initial_z = LaunchConfiguration('initial_z')

    use_ground_truth_odom_arg = DeclareLaunchArgument(
        'use_ground_truth_odom',
        default_value='false',
        description='Whether to publish odom->base_footprint directly from ground truth'
    )
    use_ground_truth_odom = LaunchConfiguration('use_ground_truth_odom')

    publish_kiss_tf_arg = DeclareLaunchArgument(
        'publish_kiss_tf',
        default_value='true',
        description='Whether KISS-ICP publishes odom->base_footprint transform directly'
    )
    publish_kiss_tf = LaunchConfiguration('publish_kiss_tf')

    use_wheel_odom_arg = DeclareLaunchArgument(
        'use_wheel_odom',
        default_value='false',
        description='Whether KISS-ICP fuses wheel odometry translation prior'
    )
    use_wheel_odom = LaunchConfiguration('use_wheel_odom')

    use_kinematic_ekf_arg = DeclareLaunchArgument(
        'use_kinematic_ekf',
        default_value='true',
        description='Whether to run Rover Kinematic Terramechanics EKF for odom->base_footprint'
    )
    use_kinematic_ekf = LaunchConfiguration('use_kinematic_ekf')

    enable_lidar_update_arg = DeclareLaunchArgument(
        'enable_lidar_update',
        default_value='true',
        description='Whether EKF performs LiDAR measurement updates'
    )
    enable_lidar_update = LaunchConfiguration('enable_lidar_update')

    gamma_observability_arg = DeclareLaunchArgument(
        'gamma_observability',
        default_value='5.0',
        description='Gain knob for eigenvalue sensitivity in Option A adaptive LiDAR fusion'
    )
    gamma_observability = LaunchConfiguration('gamma_observability')

    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout',
        default_value='0.0',
        description='Timeout to zero commanded velocity (0.0 to disable timeout for keyboard teleop)'
    )
    cmd_timeout = LaunchConfiguration('cmd_timeout')

    publish_kiss_tf_effective = PythonExpression([
        "'false' if '", use_kinematic_ekf, "' == 'true' else '", publish_kiss_tf, "'"
    ])

    # Fast-LIO 2 LiDAR-Inertial Odometry Node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        output='screen',
        parameters=[
            fast_lio_config_path,
            {'use_sim_time': True}
        ],
        remappings=[
            ('/Odometry', '/kiss/odometry'),
        ],
        condition=IfCondition(use_fast_lio),
    )

    # 1. Static map->odom fallback (used when oracle is disabled)
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', initial_z, '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=UnlessCondition(use_oracle),
    )

    # 5. Satellite Oracle Node (0.5 Hz ground truth fix on Mars)
    satellite_oracle = Node(
        package='rover_localization',
        executable='satellite_oracle_node',
        name='satellite_oracle_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'update_rate': 0.5,
            'target_model': 'leo_rover',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'oracle_topic': '/satellite/oracle_fix',
            'base_link_offset_z': 0.0,
        }],
        condition=IfCondition(use_oracle),
    )

    # 6. Global 3D Corrector (fuses 0.5 Hz oracle with EKF odom, broadcast map -> odom)
    global_3d_corrector = Node(
        package='rover_localization',
        executable='global_3d_corrector',
        name='global_3d_corrector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'oracle_topic': '/satellite/oracle_fix',
            'odom_topic': '/odometry/local',
            'publish_rate': 20.0,
            'max_trans_vel': 2.0,
            'max_yaw_vel': 2.0,
            'initial_z': initial_z,
        }],
        condition=IfCondition(use_oracle),
    )

    # 4. KISS-ICP LiDAR Odometry Node
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        remappings=[
            ('pointcloud_topic', '/points'),
            ('imu_topic', '/imu/data'),
            ('wheel_odom_topic', '/odom'),
        ],
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_footprint',
            'lidar_odom_frame': 'odom',
            'publish_odom_tf': publish_kiss_tf_effective,
            'publish_debug_clouds': False,
            'position_covariance': 0.01,
            'orientation_covariance': 0.01,
            'use_imu': True,
            'use_wheel_odom': use_wheel_odom,
            'data.deskew': False,
            'data.max_range': 15.0,
            'data.min_range': 0.35,
            'mapping.voxel_size': 0.15,
            'mapping.max_points_per_voxel': 20,
            'adaptive_threshold.min_motion_th': 0.05,
            'adaptive_threshold.initial_threshold': 0.5,
            'registration.max_num_threads': 4,
            'sliding_window_size': 20,
        }],
        condition=UnlessCondition(use_fast_lio),
    )

    # 5. Rover Kinematic Terramechanics EKF Node
    rover_kinematic_ekf_node = Node(
        package='rover_localization',
        executable='rover_kinematic_ekf',
        name='rover_kinematic_ekf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_tf': True,
            'publish_rate': 60.0,
            'cmd_vel_topic': '/cmd_vel',
            'imu_topic': '/imu/data',
            'lidar_odom_topic': '/kiss/odometry',
            'k_long_slip': 0.025,
            'k_lat_slip': 0.035,
            'k_spin_boost': 1.5,
            'cmd_timeout': cmd_timeout,
            'enable_lidar_update': enable_lidar_update,
            'gamma_observability': gamma_observability,
        }],
        condition=IfCondition(use_kinematic_ekf),
    )

    # 6. Ground truth bridge for benchmarking & visualization
    ground_truth_node = Node(
        package='rover_localization',
        executable='ground_truth_node',
        name='ground_truth_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_base_tf': use_ground_truth_odom,
        }],
    )

    return LaunchDescription([
        use_fast_lio_arg,
        use_oracle_arg,
        initial_z_arg,
        use_ground_truth_odom_arg,
        publish_kiss_tf_arg,
        use_wheel_odom_arg,
        use_kinematic_ekf_arg,
        enable_lidar_update_arg,
        gamma_observability_arg,
        cmd_timeout_arg,
        fast_lio_node,
        kiss_icp_node,
        rover_kinematic_ekf_node,
        static_map_to_odom,
        satellite_oracle,
        global_3d_corrector,
        ground_truth_node,
    ])

