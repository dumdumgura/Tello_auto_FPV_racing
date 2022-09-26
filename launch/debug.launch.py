#!/usr/bin/env python3

import os
from posixpath import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.parameter import Parameter


def generate_launch_description():
    ns = 'drone1'
    
    return LaunchDescription([
        Node(
            package='jlb_pid',
            namespace='/drone1',
            executable='controller_node',
            name='distance_controller',
            parameters=[
                # Required
                {'kp': 2.0},
                {'ki': 0.01},
                {'kd': 0.02},
                # Optional
                {'upper_limit': 0.6},
                {'lower_limit': -0.6},
                {'windup_limit': 0.001}
            ]
        )


    ])