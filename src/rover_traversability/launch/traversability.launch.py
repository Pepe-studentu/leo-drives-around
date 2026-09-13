import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rover_traversability')
    default_params_file = os.path.join(pkg, 'config', 'traversability.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Path to the traversability_node params YAML.',
    )

    return LaunchDescription([
        params_file_arg,
        Node(
            package='rover_traversability',
            executable='traversability_node',
            name='traversability_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
