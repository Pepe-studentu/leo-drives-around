"""Toy-world first-principles height-map visualization -- stage 3 of 3.

Only the height-map viz node and RViz. Run AFTER both calibration_sim.launch.py
(Gazebo + robot + bridge) and rover_localization's localization.launch.py
(local+global EKF + navsat_transform, publishing map -> base_footprint from
real GPS+IMU+wheel-odom fusion) are already up and settled -- matching the
staged sim -> localization -> rest workflow already used for Mars Yard,
instead of starting everything simultaneously.

  ros2 launch gazebo_stuff calibration_sim.launch.py
  ros2 launch rover_localization localization.launch.py
  ros2 launch gazebo_stuff calibration_drive.launch.py
  ros2 launch leo_teleop key_teleop.launch.xml   # in another terminal, to drive
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_stuff = get_package_share_directory('gazebo_stuff')
    rviz_config_path = os.path.join(pkg_gazebo_stuff, 'rviz', 'height_map_viz.rviz')

    height_map_viz = Node(
        package='rover_traversability', executable='height_map_viz_node',
        name='height_map_viz_node', output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([height_map_viz, rviz])
