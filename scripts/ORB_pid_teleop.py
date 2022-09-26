#from _typeshed import Self
import math
from time import sleep
import numpy as np
from regex import D
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn



class pid_teleop(Node):

    def __init__(self):
        super().__init__('pid_teleop')
        # Create turtle2 velocity publisher
        self.angle_sub = self.create_subscription(Float64, '/drone1/pid/az/control_effort', self.call_back_az,5)
        self.dish_sub = self.create_subscription(Float64, '/drone1/pid/x/control_effort', self.call_back_x,5)
        self.disv_sub = self.create_subscription(Float64, '/drone1/pid/y/control_effort', self.call_back_y,5)
        self.disv_sub = self.create_subscription(Float64, '/drone1/pid/z/control_effort', self.call_back_z,5)
        self.disv_sub = self.create_subscription(Float64, '/drone1/pid/z/state', self.call_back_z_state,5)
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel',qos_profile=10)

        enable_service= self.create_service(SetBool,'/drone1/pid/teleop_enbale',self.enable_callback)
        # Call on_timer function every second
        self.timer = self.create_timer(1/60.0, self.on_timer)   
        self.vel = Twist()
        self.state_z = 0.0
        self.enable = False
        #self.enable = True
   
    def enable_callback(self,request,response):
        self.enable = request.data
        response.success = True
        return response
    
    

    def call_back_z_state(self,msg):
        self.state_z = msg.data
        #print("angle")

    def call_back_az(self,msg):
        self.vel.angular.z = msg.data
        #print("angle")

    def call_back_x(self,msg):
        self.vel.linear.x = msg.data
        pass
    def call_back_y(self,msg):
        self.vel.linear.y = msg.data
        pass

    def call_back_z(self,msg):
        self.vel.linear.z = msg.data
        pass

    def on_timer(self):
        self.local_vel = Twist()
        self.local_vel.angular.z = self.vel.angular.z
        #self.local_vel.linear.x = self.vel.linear.x * np.cos(float(self.state_z)) + self.vel.linear.y * np.sin(float(self.state_z))
        #self.local_vel.linear.y =-(self.vel.linear.x * np.sin(float(self.state_z)) - self.vel.linear.y * np.cos(float(self.state_z)))

        self.local_vel.linear.x = self.vel.linear.x
        self.local_vel.linear.y = self.vel.linear.y
        self.local_vel.linear.z = self.vel.linear.z
        if self.enable:
            self.vel_pub.publish(self.local_vel)
        #print("receiving msg now")
    
def main():
    rclpy.init()
    node = pid_teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()