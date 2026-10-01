"""Mars Yard end-to-end bringup: sim + localization + traversability + Nav2.

Chains the existing, independently-working launch files so the whole
autonomous stack comes up with one command:

  ros2 launch rover_navigation marsyard_bringup.launch.py

Arguments:
  rviz:=true|false       open RViz with the Mars Yard view (default true)
  headless:=true|false   run Gazebo without its GUI window (default true)

Once Nav2 reports active, run the mission benchmark separately:

  ros2 run rover_navigation mission_runner
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_stuff = get_package_share_directory('gazebo_stuff')
    pkg_rover_localization = get_package_share_directory('rover_localization')
    pkg_rover_traversability = get_package_share_directory('rover_traversability')
    pkg_rover_navigation = get_package_share_directory('rover_navigation')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz with the Mars Yard view (robot, ground truth, costmaps, plans)')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo without its GUI window')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_stuff, 'launch', 'rover_sim.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, 'launch', 'localization.launch.py')),
    )

    traversability = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_traversability, 'launch', 'traversability.launch.py')),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_navigation, 'launch', 'navigation.launch.py')),
    )

    # Ground-truth terrain mesh shown in RViz
    marsyard_mesh_pub = Node(
        package='gazebo_stuff',
        executable='marsyard_mesh_publisher',
        name='marsyard_mesh_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mesh_resource': 'package://gazebo_stuff/models/marsyard2022_ground_truth_solid.obj',
            'frame_id': 'map',
            'color_r': 0.76,
            'color_g': 0.50,
            'color_b': 0.32,
            'color_a': 1.0,
            'publish_rate': 0.5,
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_gazebo_stuff, 'rviz', 'height_map_viz.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg, headless_arg,
        sim, localization, traversability, navigation,
        marsyard_mesh_pub, rviz,
    ])
