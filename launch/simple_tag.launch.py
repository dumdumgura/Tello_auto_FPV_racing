"""Simulate a Tello drone"""

import os
from ament_index_python.constants import AMENT_PREFIX_PATH_ENV_VAR
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from posixpath import join


def generate_launch_description(): 
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    camerainfo_path = os.path.join('/home/hongli/tello_ws/src/tello_mod/config','webcam.yaml')
    apriltag_ros_path = os.path.join(get_package_share_directory('apriltag_ros'),'launch','tag_36h11_all.launch.py')
    print(camerainfo_path)
    return LaunchDescription([
        
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(get_package_share_directory('fpv_racing_gazebo'), 'models')),
        #Launch Gazebo, loading tello.world
        #ExecuteProcess(cmd=[
        #    'gazebo',
        #    '--verbose',
        #    '-s', 'libgazebo_ros_init.so',  # Publish /clock
        #    '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
        #    world_path
        #], output='screen'),

        # Spawn tello.urdf
        # Node(package='tello_gazebo', executable='inject_entity.py', output='screen',arguments=[urdf_path, '0', '0', '1', '0']),

        # Publish static transforms
        # Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',arguments=[urdf_path]),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen', namespace=ns),
        Node(package='tello_driver',executable='tello_driver_main', output='screen',namespace=ns,parameters=[{"camera_info_path": camerainfo_path}]),
        # Node(package='tello_mod',executable='image_receive',namespace=ns),
        Node(package='tello_mod',executable='tello_teleop', output='screen',namespace=ns),

        
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(apriltag_ros_path)
        #),
        
        Node(package='tello_mod',executable='base_tf2_broadcaster'), #tf would also be influenced by namespace.
        Node(package='tello_mod',executable='tello_tf'),
        Node(package='tello_mod',executable='pid_pub',namespace='drone1/pid/'),
        Node(package='tello_mod',executable='pid_teleop',namespace='drone1/pid/'),
        #Node(package='tello_mod',executable='graph',namespace=ns),

        Node(
            package='jlb_pid',
            namespace='/drone1/pid/angle',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 0.5},
                {'ki': 0.01},
                {'kd': 0.02},
                # Optional
                {'upper_limit': 0.6},
                {'lower_limit': -0.6},
                {'windup_limit': 0.001},
                {'update_rate': float(30.0)}
            ]
        ),

        Node(
            package='jlb_pid',
            namespace='/drone1/pid/dis_h',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 0.25},
                {'ki': 0.1},
                {'kd': 0.0},
                # Optional
                {'upper_limit': 1.0},
                {'lower_limit': -1.0},
                {'windup_limit': 0.001},
                {'update_rate': float(30.0)}
            ]
        ),
        Node(
            package='jlb_pid',
            namespace='/drone1/pid/dis_v',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 0.5},
                {'ki': 0.01},
                {'kd': 0.02},
                # Optional
                {'upper_limit': 0.6},
                {'lower_limit': -0.6},
                {'windup_limit': 0.001},
                {'update_rate': float(30.0)}
            ]
        )
        #Node(package='tello_mod',executable='tello_tf_listener'),
        # Joystick controller, generates /namespace/cmd_vel messages
        # Node(package='tello_driver', executable='tello_joy_main', output='screen', namespace=ns),
    ])
    
'''
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),
'''