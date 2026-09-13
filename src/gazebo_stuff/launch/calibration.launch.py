"""Traversability calibration rig.

Runs the tilted-lidar rover STATIONARY on the calibration world's pad and pins
map -> base_footprint with an exact static transform (no EKF, no GPS, no drift),
so map coordinates == world coordinates. Then runs the traversability node.
Validate with:  ros2 run rover_traversability calib_checker
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# rover spawns at the origin on FLAT ground (top z = 0); base_footprint is the
# ground-contact frame, so map -> base_footprint is exactly identity.
GROUND_Z = 0.0


def generate_launch_description():
    pkg_gazebo_stuff = get_package_share_directory('gazebo_stuff')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_trav = get_package_share_directory('rover_traversability')

    world_path = os.path.join(pkg_gazebo_stuff, 'worlds', 'calibration.sdf')
    urdf_file = os.path.join(pkg_gazebo_stuff, 'urdf', 'rover_with_lidar.urdf.xacro')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_rover = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'leo_rover', '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.30'],
    )

    # EXACT ground truth: rover is static on flat pad at the origin.
    static_map = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['--x', '0', '--y', '0', '--z', str(GROUND_Z),
                   '--frame-id', 'map', '--child-frame-id', 'base_footprint'],
        parameters=[{'use_sim_time': True}],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/imu/data_raw', '/imu/data'), ('/points/points', '/points')],
        output='screen',
    )

    traversability = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_trav, 'launch', 'traversability.launch.py')),
    )

    return LaunchDescription([
        gazebo, robot_state_publisher, spawn_rover, static_map, bridge, traversability,
    ])
