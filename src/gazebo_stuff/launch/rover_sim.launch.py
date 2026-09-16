import os                                                                                                                                                                  
from ament_index_python.packages import get_package_share_directory                                                                                                        
from launch import LaunchDescription                                                                                                                                       
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable                                                                         
from launch.launch_description_sources import PythonLaunchDescriptionSource                                                                                                
from launch.substitutions import Command, LaunchConfiguration, PythonExpression                                                                                                              
from launch_ros.actions import Node                                                                                                                                        
from launch_ros.parameter_descriptions import ParameterValue                                                                                                               
                                                                                                                                                                            
                                                                                                                                                                            
def generate_launch_description():                                                                                                                                         
    pkg_gazebo_stuff = get_package_share_directory('gazebo_stuff')                                                                                                         
    pkg_leo_worlds = get_package_share_directory('leo_gz_worlds')                                                                                                          
    pkg_leo_description = get_package_share_directory('leo_description')                                                                                                   
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')                                                                                                             
                                                                                                                                                                            
    # Path to the 3D Marsyard World                                                                                                                                        
    world_path = os.path.join(pkg_gazebo_stuff, 'worlds', 'marsyard2022_gps.sdf')                                                                                                
                                                                                                                                                                            
    # 1. Set Gazebo Resource Path so it finds the 3D Mars mesh automatically                                                                                               
    models_path = os.path.join(pkg_leo_worlds, 'models')                                                                                                                   
    set_gz_resource_path = SetEnvironmentVariable(                                                                                                                         
        name='GZ_SIM_RESOURCE_PATH',                                                                                                                                       
        value=f'{models_path}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'                                                                                                
    )                                                                                                                                                                      
                                                                                                                                                                            
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Whether to run Gazebo in headless mode (-s server only, no GUI window)'
    )
    headless = LaunchConfiguration('headless')

    gz_args = PythonExpression([
        f"'-r -s {world_path}' if '", headless, "' == 'true' else '-r {world_path}'"
    ])

    # 2. Start Gazebo Sim with Marsyard                                                                                                                                    
    gazebo = IncludeLaunchDescription(                                                                                                                                     
        PythonLaunchDescriptionSource(                                                                                                                                     
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')                                                                                                     
        ),                                                                                                                                                                 
        launch_arguments={'gz_args': gz_args}.items(),                                                                                                          
    )                                                                                                                                                                      
                                                                                                                                                                            
    # 3. Process the 3D LiDAR Rover URDF                                                                                                                                   
    urdf_file = os.path.join(pkg_gazebo_stuff, 'urdf', 'rover_with_lidar.urdf.xacro')                                                                                      
    robot_description_config = ParameterValue(                                                                                                                             
        Command(['xacro ', urdf_file]),                                                                                                                                    
        value_type=str                                                                                                                                                     
    )                                                                                                                                                                      
                                                                                                                                                                            
    # 4. Robot State Publisher (TF)                                                                                                                                        
    robot_state_publisher = Node(                                                                                                                                          
        package='robot_state_publisher',                                                                                                                                   
        executable='robot_state_publisher',                                                                                                                                
        output='both',                                                                                                                                                     
        parameters=[{                                                                                                                                                      
            'robot_description': robot_description_config,                                                                                                                 
            'use_sim_time': True                                                                                                                                           
        }]                                                                                                                                                                 
    )                                                                                                                                                                      
                                                                                                                                                                            
    # 5. Spawn the rover slightly elevated so it lands gently on the Mars ground                                                                                           
    spawn_rover = Node(                                                                                                                                                    
        package='ros_gz_sim',                                                                                                                                              
        executable='create',                                                                                                                                               
        output='screen',                                                                                                                                                   
        arguments=[                                                                                                                                                        
            '-name', 'leo_rover',                                                                                                                                          
            '-topic', 'robot_description',                                                                                                                                 
            '-x', '-10.09',                                                                                                                                                   
            '-y', '9.33',                                                                                                                                                   
            '-z', '1.65',      # Spawns onto the flat Mars surface at Base Station
            '-Y', '0.0'
        ]                                                                                                                                                                  
    )                                                                                                                                                                      
                                                                                                                                                                            
    # 6. Bridge all ROS 2 <-> Gazebo topics (including 3D PointCloud)                                                                                                      
    bridge = Node(                                                                                                                                                         
        package='ros_gz_bridge',                                                                                                                                           
        executable='parameter_bridge',                                                                                                                                     
        parameters=[{'use_sim_time': True}],                                                                                                                               
        arguments=[                                                                                                                                                        
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',                                                                                                                
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',                                                                                                              
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',                                                                                                                
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',                                                                                                               
            '/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/world/leo_marsyard/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/imu/data_raw', '/imu/data'),
            ('/points/points', '/points'),
            ('/world/leo_marsyard/dynamic_pose/info', '/gazebo/dynamic_pose'),
        ],
        output='screen'
    )

    return LaunchDescription([
        headless_arg,
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_rover,
        bridge,
    ])
