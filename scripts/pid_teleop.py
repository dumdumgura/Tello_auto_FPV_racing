#from _typeshed import Self
import math
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
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel',qos_profile=10)

        enable_service= self.create_service(SetBool,'teleop_enbale',self.enable_callback)
        # Call on_timer function every second
        self.timer = self.create_timer(1/200, self.on_timer)   
        self.vel = Twist()
        self.enable = False
    
    def enable_callback(self,request,response):
        self.enable = request.data
        response.success = True
        return response

    def call_back_az(self,msg):
        self.vel.angular.z = -msg.data
        #print("angle")

    def call_back_x(self,msg):
        self.vel.linear.x = -msg.data/2.8
        pass
    def call_back_y(self,msg):
        self.vel.linear.y = msg.data*0.8
        pass

    def call_back_z(self,msg):
        self.vel.linear.z = -msg.data/2.0
        pass

    def on_timer(self):
        if self.enable:
            self.vel_pub.publish(self.vel)
        #print("receiving msg now")
    
def main():
    rclpy.init()
    node = pid_teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()