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
    #kp = 1.8
    kp = 3.0
    #world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    #urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    camerainfo_path = os.path.join('/home/hongli/tello_ws/src/tello_mod/config','webcam.yaml')
    #apriltag_ros_path = os.path.join(get_package_share_directory('apriltag_ros'),'tag_16h5_all.launch.py')
    print(camerainfo_path)
    return LaunchDescription([
       # SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(get_package_share_directory('fpv_racing_gazebo'), 'models')),
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
        #Node(package='joy', executable='joy_node', output='screen', namespace=ns),
        
        Node(package='tello_driver',executable='tello_driver_main', output='screen',namespace=ns,parameters=[{"camera_info_path": camerainfo_path}]),
        Node(package='tello_mod',executable='base_tf2_broadcaster'), #tf would also be influenced by namespace.

        Node(package='tello_mod',executable='image_receive_new',namespace=ns),
        #Node(package='tello_mod',executable='tello_teleop', output='screen',namespace=ns),
        #Node(package='tello_mod',executable='strategy', output='screen',namespace=ns),
        
        Node(package='tello_mod',executable='gate_waypoint_pub',output='screen'),
        Node(package='tello_mod',executable='slow_rotate_server',namespace=ns),
        Node(package='tello_mod',executable='polynomial_path_planner',namespace=ns,output='screen'),
        Node(package='tello_mod',executable='ORB_pid_pub2', output='screen',namespace=ns),
        Node(package='tello_mod',executable='ORB_pid_teleop', output='screen',namespace=ns),
        Node(package='tello_mod',executable='sub', output='screen',namespace=ns),

        Node(
            package='jlb_pid',
            namespace='/drone1/pid/az',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 0.1*kp},
                {'ki': 0.00},
                {'kd': 0.00},
                # Optional
                {'upper_limit': 0.6},
                {'lower_limit': -0.6},
                {'windup_limit': 0.001},
                {'update_rate': float(60.0)}
            ]
        ),

        Node(
            package='jlb_pid',
            namespace='/drone1/pid/x',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': kp},
                {'ki': 0.2*kp},
                {'kd': 0.00},
                # Optional
                {'upper_limit': 1.0},
                {'lower_limit': -1.0},
                {'windup_limit': 0.001},
                {'update_rate': float(60.0)}
            ]
        ),

        Node(
            package='jlb_pid',
            namespace='/drone1/pid/y',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': kp},
                {'ki':  0.2*kp},
                {'kd': 0.00},
                # Optional
                {'upper_limit': 1.0},
                {'lower_limit': -1.0},
                {'windup_limit': 0.001},
                {'update_rate': float(60.0)}
            ]
        ),
        
        
        Node(
            package='jlb_pid',
            namespace='/drone1/pid/z',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 3*kp},
                {'ki': kp},
                {'kd': 0.00},
                # Optional
                {'upper_limit': 1.0},
                {'lower_limit': -1.0},
                {'windup_limit': 0.001},
                {'update_rate': float(10.0)}
            ]
        ),

        

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