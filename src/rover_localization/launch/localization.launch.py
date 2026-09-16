import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_localization = get_package_share_directory('rover_localization')
    ekf_config_path = os.path.join(pkg_rover_localization, 'config', 'ekf.yaml')

    use_oracle_arg = DeclareLaunchArgument(
        'use_oracle',
        default_value='true',
        description='Whether to use low-rate 3D satellite oracle and global 3D corrector'
    )
    use_oracle = LaunchConfiguration('use_oracle')

    initial_z_arg = DeclareLaunchArgument(
        'initial_z',
        default_value='1.135',
        description='Initial ground elevation datum for map->odom Z offset (1.135 for Base Station)'
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
        description='Whether to run Rover Kinematic EKF for odom->base_footprint'
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
        description='Eigenvalue sensitivity gain for adaptive LiDAR fusion'
    )
    gamma_observability = LaunchConfiguration('gamma_observability')

    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout',
        default_value='0.0',
        description='Timeout to zero commanded velocity (0.0 to disable)'
    )
    cmd_timeout = LaunchConfiguration('cmd_timeout')

    publish_kiss_tf_effective = PythonExpression([
        "'false' if '", use_kinematic_ekf, "' == 'true' else '", publish_kiss_tf, "'"
    ])

    # Static map->odom fallback (used when oracle is disabled)
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', initial_z, '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=UnlessCondition(use_oracle),
    )

    # Satellite Oracle Node (Ground truth periodic reference fix)
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

    # Global 3D Corrector (Fuses periodic oracle with local odometry, broadcasts map -> odom)
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

    # KISS-ICP 3D LiDAR Odometry Node
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
    )

    # Rover Kinematic Terramechanics EKF Node
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

    # Ground truth bridge for benchmarking and evaluation
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
        use_oracle_arg,
        initial_z_arg,
        use_ground_truth_odom_arg,
        publish_kiss_tf_arg,
        use_wheel_odom_arg,
        use_kinematic_ekf_arg,
        enable_lidar_update_arg,
        gamma_observability_arg,
        cmd_timeout_arg,
        kiss_icp_node,
        rover_kinematic_ekf_node,
        static_map_to_odom,
        satellite_oracle,
        global_3d_corrector,
        ground_truth_node,
    ])
