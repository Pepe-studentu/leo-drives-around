"""Mars Yard end-to-end bringup: sim + localization + traversability + Nav2.

Chains the existing, independently-working launch files so the whole
autonomous stack comes up with one command:

  ros2 launch rover_navigation marsyard_bringup.launch.py

Once Nav2 reports active (and navsat_transform's datum has settled), run the
mission benchmark separately:

  ros2 run rover_navigation mission_runner
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_gazebo_stuff = get_package_share_directory('gazebo_stuff')
    pkg_rover_localization = get_package_share_directory('rover_localization')
    pkg_rover_traversability = get_package_share_directory('rover_traversability')
    pkg_rover_navigation = get_package_share_directory('rover_navigation')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_stuff, 'launch', 'rover_sim.launch.py')),
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

    return LaunchDescription([sim, localization, traversability, navigation])
