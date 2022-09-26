#from _typeshed import Self
from cmath import sqrt
import math
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('tello_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        #self.declare_parameter('target_frame', 'marker3')
        
        #self.target_frame = self.get_parameter(
        #    'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        #self.publisher = self.create_publisher(Twist, '/drone1/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(0.2, self.on_timer)   
        
        self.src_frame = 'map'
        self.target_frame = 'camera_link'
        
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        try:
            now = rclpy.time.Time()
            self.trans = self.tf_buffer.lookup_transform(
                self.src_frame,
                self.target_frame,
                now)
            print(self.trans)
        
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform from {self.src_frame} to {self.target_frame} :{ex}')
          
            return

 

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()