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
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_localization = get_package_share_directory('rover_localization')
    ekf_config_path = os.path.join(pkg_rover_localization, 'config', 'ekf.yaml')

    # 1. Local EKF (odom -> base_footprint)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')],
    )

    # 2. Global EKF (map -> odom)
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')],
    )

    # 3. Navsat transform (GPS fix -> /odometry/gps in map frame)
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps'),
        ],
    )

    # 4. Ground truth bridge for benchmarking & visualization
    ground_truth_node = Node(
        package='rover_localization',
        executable='ground_truth_node',
        name='ground_truth_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        ekf_local,
        ekf_global,
        navsat_transform,
        ground_truth_node,
    ])
