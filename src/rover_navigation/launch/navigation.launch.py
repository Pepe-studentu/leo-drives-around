import os
from ament_index_python.packages import get_package_share_directory     
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 


def generate_launch_description():  
    pkg_rover_navigation = get_package_share_directory('rover_navigation')       
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')      
    params_file = os.path.join(pkg_rover_navigation, 'config', 'nav2_params.yaml')        

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch default Nav2 RViz'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for controller_server (info, debug)'
    )
    log_level = LaunchConfiguration('log_level')

    # The 6 nodes managed by our Lifecycle Manager       
    lifecycle_nodes = [    
        'controller_server',
        'smoother_server', 
        'planner_server',
        'behavior_server',
        'bt_navigator',    
        'waypoint_follower',        
    ]    

    # 1. Controller Server (Regulated Pure Pursuit)   
    controller_server = Node(       
        package='nav2_controller',  
        executable='controller_server',      
        output='screen',   
        parameters=[params_file],   
        remappings=[('cmd_vel', '/cmd_vel')],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # 2. Smoother Server
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        output='screen',
        parameters=[params_file]
    )

    # 3. Planner Server (Global Navfn / A* path planner)    
    planner_server = Node( 
        package='nav2_planner',     
        executable='planner_server',
        output='screen',   
        parameters=[params_file]    
    )    

    # 4. Behavior Server (Spin, Backup recoveries)    
    behavior_server = Node(
        package='nav2_behaviors',   
        executable='behavior_server',        
        output='screen',   
        parameters=[params_file]    
    )    

    # 5. BT Navigator (Coordinates navigation goals)  
    bt_navigator = Node(   
        package='nav2_bt_navigator',
        executable='bt_navigator',  
        output='screen',   
        parameters=[params_file]    
    )    

    # 6. Waypoint Follower (Iterates through coordinate waypoints)      
    waypoint_follower = Node(       
        package='nav2_waypoint_follower',    
        executable='waypoint_follower',      
        output='screen',   
        parameters=[params_file]    
    )    

    # 7. Lifecycle Manager (Bootstraps all nodes to Active state)     
    lifecycle_manager = Node(       
        package='nav2_lifecycle_manager',    
        executable='lifecycle_manager',      
        name='lifecycle_manager_navigation', 
        output='screen',   
        parameters=[{      
            'use_sim_time': True,   
            'autostart': True,      
            'node_names': lifecycle_nodes    
        }]        
    )    

    # 8. Optional RViz2 Navigation View (off by default)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([      
        use_rviz_arg,
        log_level_arg,
        controller_server, 
        planner_server,
        smoother_server,   
        behavior_server,   
        bt_navigator,      
        waypoint_follower, 
        lifecycle_manager, 
        rviz_node,
    ]) 