"""Toy-world simulation only: Gazebo, robot, and the ROS<->GZ bridge.

Stage 1 of 3 -- mirrors the Mars Yard workflow (sim, then localization, then
the rest), instead of bundling everything into one simultaneous launch. Start
this first, wait for it to settle, then bring up localization, then the rest:

  ros2 launch gazebo_stuff calibration_sim.launch.py
  ros2 launch rover_localization localization.launch.py   # once sim is up
  ros2 launch gazebo_stuff calibration_drive.launch.py     # once localization is up
  ros2 launch leo_teleop key_teleop.launch.xml             # to drive
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo_stuff = get_package_share_directory('gazebo_stuff')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

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

    return LaunchDescription([
        gazebo, robot_state_publisher, spawn_rover, bridge,
    ])
