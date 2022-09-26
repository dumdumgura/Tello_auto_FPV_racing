#from _typeshed import Self
import math
from cmath import pi
from re import S
from sre_parse import State
import rclpy
from rclpy.node import Node
from turtle import st
from geometry_msgs.msg import PoseStamped,Point,Pose
from nav_msgs.msg import Path
from rclpy.node import Node
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Bool


class path_planner(Node):

    def __init__(self):
        super().__init__('sub')
        # Create turtle2 velocity publisher
        self.goal_sub = self.create_subscription(PoseStamped, '/waypoint', self.call_back_goal,10)
        self.path_pub =self.create_publisher(Path,'/planning_path_global',5)

        self.pathMsg = Path() 
        self.pathMsg.header.frame_id = 'map'
        
        self.gate_center = 0.032



    def call_back_goal(self,msg):
        self.pathMsg.header.frame_id = 'map'
        self.pathMsg.poses.append(msg)
        self.path_pub.publish(self.pathMsg)



def main():
    rclpy.init()
    node = path_planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()