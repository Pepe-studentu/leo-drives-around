import os
from ament_index_python.packages import get_package_share_directory     
from launch import LaunchDescription
from launch_ros.actions import Node 
 
 
def generate_launch_description():  
    pkg_rover_navigation = get_package_share_directory('rover_navigation')       
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')      
    params_file = os.path.join(pkg_rover_navigation, 'config', 'nav2_params.yaml')        
 
    # The exact 5 nodes managed by our Lifecycle Manager       
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
        remappings=[('cmd_vel', '/cmd_vel')] 
    )
    
    smoother_server = Node(
            package='nav2_smoother',
            executable='smoother_server',
            output='screen',
            parameters=[params_file]
        )
    
 
    # 2. Planner Server (Global A* / Dijkstra path planner)    
    planner_server = Node( 
        package='nav2_planner',     
        executable='planner_server',
        output='screen',   
        parameters=[params_file]    
    )    
 
    # 3. Behavior Server (Spin, Backup recoveries)    
    behavior_server = Node(
        package='nav2_behaviors',   
        executable='behavior_server',        
        output='screen',   
        parameters=[params_file]    
    )    
 
    # 4. BT Navigator (Coordinates navigation goals)  
    bt_navigator = Node(   
        package='nav2_bt_navigator',
        executable='bt_navigator',  
        output='screen',   
        parameters=[params_file]    
    )    
 
    # 5. Waypoint Follower (Iterates through coordinate waypoints)      
    waypoint_follower = Node(       
        package='nav2_waypoint_follower',    
        executable='waypoint_follower',      
        output='screen',   
        parameters=[params_file]    
    )    
 
    # 6. Lifecycle Manager (Bootstraps all 5 nodes to Active state)     
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
 
    # 7. RViz2 Navigation View
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. ETH Zurich Ground Plane / Slope Segmentation Node
    pkg_segmentation = get_package_share_directory('linefit_ground_segmentation_ros')
    segmentation_params = os.path.join(pkg_segmentation, 'launch', 'segmentation_params.yaml')

    ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        output='screen',
        parameters=[segmentation_params, {'use_sim_time': True}]
    )

    return LaunchDescription([      
        ground_segmentation_node,
        controller_server, 
        planner_server,
        smoother_server,   
        behavior_server,   
        bt_navigator,      
        waypoint_follower, 
        lifecycle_manager, 
        rviz_node,
    ]) 