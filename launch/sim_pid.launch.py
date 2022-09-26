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
    world_path = os.path.join(get_package_share_directory('fpv_racing_gazebo'), 'worlds','labor_worlds','2artag_1hsv.world')
    
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
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',arguments=[urdf_path, '-5', '0', '1', '0']),
        #Node(package='tello_gazebo', executable='inject_entity.py', output='screen',arguments=[urdf_path, '20', '20', '1', '0']),

        # Publish static transforms
        #Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        #     arguments=[urdf_path]),

       

      
    
        # image_processing 
        Node(package='tello_mod',executable='image_receive',namespace=ns),
        
        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen', namespace=ns),
        # Joystick controller, generates /namespace/cmd_vel messages
        # Node(package='tello_mod',executable='tello_teleop', output='screen',namespace=ns),

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(apriltag_ros_path)
        ),

        #Node(package='tello_mod',executable='tello_tf_listener', output='screen',parameters=[{'use_sim_time': True}]),
        Node(package='tello_mod',executable='base_tf2_broadcaster',parameters=[{'use_sim_time': True}]), #tf would also be influenced by namespace.
        Node(package='tello_mod',executable='tello_tf',parameters=[{'use_sim_time': True}]),

        Node(package='tello_mod',executable='blind_fly_server',namespace=ns),
        Node(package='tello_mod',executable='slow_rotate_server',namespace=ns),
        Node(package='tello_mod',executable='slow_forward_server',namespace=ns),
        Node(package='tello_mod',executable='strategy',parameters=[{'use_sim_time': True}]),

        Node(package='tello_mod',executable='pid_pub',namespace='drone1/pid/'),
        Node(package='tello_mod',executable='pid_teleop',namespace='drone1/pid/'),
        #Node(package='tello_mod',executable='graph',namespace=ns),


        Node(
            package='jlb_pid',
            namespace='/drone1/pid/az',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 0.1},
                {'ki': 0.01},
                {'kd': 0.05},
                # Optional
                {'upper_limit': 0.2},
                {'lower_limit': -0.2},
                {'windup_limit': 0.001},
                {'update_rate': float(30.0)}
            ]
        ),

        Node(
            package='jlb_pid',
            namespace='/drone1/pid/x',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 0.05},
                {'ki': 0.01},
                {'kd': 0.02},
                # Optional
                {'upper_limit': 0.3},
                {'lower_limit': -0.3},
                {'windup_limit': 0.001},
                {'update_rate': float(30.0)}
            ]
        ),
        Node(
            package='jlb_pid',
            namespace='/drone1/pid/y',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 0.2},
                {'ki': 0.01},
                {'kd': 0.02},
                # Optional
                {'upper_limit': 0.5},
                {'lower_limit': -0.5},
                {'windup_limit': 0.001},
                {'update_rate': float(30.0)}
            ]
        ),
        Node(
            package='jlb_pid',
            namespace='/drone1/pid/z',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 0.15},
                {'ki': 0.01},
                {'kd': 0.02},
                # Optional
                {'upper_limit': 0.2},
                {'lower_limit': -0.2},
                {'windup_limit': 0.001},
                {'update_rate': float(30.0)}
            ]
        ),
    ])
