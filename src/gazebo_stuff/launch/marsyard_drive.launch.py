"""Mars Yard real-time height-map visualization and interactive driving in RViz.

Launches:
1. traversability_node (publishes /height_map/cloud and /height_map/raw_cloud)
2. rviz2 with height_map_viz.rviz configuration
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_stuff = get_package_share_directory('gazebo_stuff')
    rviz_config_path = os.path.join(pkg_gazebo_stuff, 'rviz', 'height_map_viz.rviz')

    height_map_viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_traversability'),
                         'launch', 'traversability.launch.py')),
    )

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
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([marsyard_mesh_pub, height_map_viz, rviz])
