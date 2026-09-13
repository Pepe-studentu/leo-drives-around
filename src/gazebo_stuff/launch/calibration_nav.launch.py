"""Toy-world end-to-end navigation testing (dynamic robot, no GPS).

Unlike calibration.launch.py (robot pinned static for the analytic layer
checker), here the robot actually drives: wheel odometry is bridged, a
local EKF fuses odom+IMU into odom->base_footprint, and a static map->odom
identity stands in for the missing GPS/navsat chain (map frame == world
frame in this world, exactly like the static-pin setup did). Traversability
and the full Nav2 stack (with RViz) are brought up on top, so you can drive
the robot with 2D Nav Goal and watch the planner react to the traversability
costmap.

  ros2 launch gazebo_stuff calibration_nav.launch.py
  ros2 run rover_traversability nav_e2e_checker   # optional pass/fail observer
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
# ground-contact frame, so map -> odom starts out an exact identity too.
GROUND_Z = 0.0


def generate_launch_description():
    pkg_gazebo_stuff = get_package_share_directory('gazebo_stuff')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_rover_localization = get_package_share_directory('rover_localization')
    pkg_trav = get_package_share_directory('rover_traversability')
    pkg_rover_navigation = get_package_share_directory('rover_navigation')

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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, 'launch', 'localization.launch.py')),
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/world/calibration/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/imu/data_raw', '/imu/data'),
            ('/points/points', '/points'),
            ('/world/calibration/dynamic_pose/info', '/gazebo/dynamic_pose'),
        ],
        output='screen',
    )

    traversability_params = os.path.join(
        pkg_trav, 'config', 'traversability_toyworld.yaml')
    traversability = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_trav, 'launch', 'traversability.launch.py')),
        launch_arguments={'params_file': traversability_params}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_navigation, 'launch', 'navigation.launch.py')),
    )

    return LaunchDescription([
        gazebo, robot_state_publisher, spawn_rover, localization,
        bridge, traversability, navigation,
    ])
