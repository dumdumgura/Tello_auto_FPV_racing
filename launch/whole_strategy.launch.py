"""Simulate a Tello drone"""

import os
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ns = 'drone1'
    #world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    world_path = os.path.join(get_package_share_directory('fpv_racing_gazebo'), 'worlds','labor_worlds','4artag.world')
    
    apriltag_ros_path = os.path.join(get_package_share_directory('apriltag_ros'),'launch','tag_36h11_all.launch.py')
    
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    return LaunchDescription([
        # Launch Gazebo, loading tello.world
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(get_package_share_directory('fpv_racing_gazebo'), 'models')),
        
        #DeclareLaunchArgument(
        #    'use_sim_time',
        #    default_value='true',
        #),

        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Spawn tello.urdf
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',arguments=[urdf_path, '0', '0', '1', '0']),

        # Publish static transforms
        #Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        #     arguments=[urdf_path]),

       

      
    
        # image_processing 
        Node(package='tello_mod',executable='image_receive',namespace=ns),
        
        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen', namespace=ns),
        # Joystick controller, generates /namespace/cmd_vel messages
        Node(package='tello_mod',executable='tello_teleop', output='screen',namespace=ns),

        
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(apriltag_ros_path)
        #),

        #Node(package='tello_mod',executable='tello_tf_listener', output='screen',parameters=[{'use_sim_time': True}]),
        
        Node(package='tello_mod',executable='base_tf2_broadcaster',parameters=[{'use_sim_time': True}]), #tf would also be influenced by namespace.
        Node(package='tello_mod',executable='tello_tf',parameters=[{'use_sim_time': True}]),

        # Node(package='tello_mod',executable='blind_fly_server',namespace=ns),
        # Node(package='tello_mod',executable='slow_rotate_server',namespace=ns),
        # Node(package='tello_mod',executable='strategy',parameters=[{'use_sim_time': True}]),

        


        
    ])
