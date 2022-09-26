#from _typeshed import Self
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('pid_pub')
        # Create turtle2 velocity publisher
        self.error_sub = self.create_subscription(Twist, '/drone1/error_status', self.call_back,10)
        #self.error_sub = self.create_subscription(Float64, '/drone1/pid/angle/control_effort', self.call_back_output,10)

        self.enable_angle_pub = self.create_publisher(Bool, '/drone1/pid/az/enable', 5)
        self.setpoint_angle_pub = self.create_publisher(Float64, '/drone1/pid/az/setpoint', 5)
        self.state_angle_pub = self.create_publisher(Float64, '/drone1/pid/az/state', 5)

        self.enable_dish_pub = self.create_publisher(Bool, '/drone1/pid/x/enable', 5)
        self.setpoint_dish_pub = self.create_publisher(Float64, '/drone1/pid/x/setpoint', 5)
        self.state_dish_pub = self.create_publisher(Float64, '/drone1/pid/x/state', 5)

        self.enable_disv_pub = self.create_publisher(Bool, '/drone1/pid/y/enable', 5)
        self.setpoint_disv_pub = self.create_publisher(Float64, '/drone1/pid/y/setpoint', 5)
        self.state_disv_pub = self.create_publisher(Float64, '/drone1/pid/y/state', 5)

        self.enable_disz_pub = self.create_publisher(Bool, '/drone1/pid/z/enable', 5)
        self.setpoint_disz_pub = self.create_publisher(Float64, '/drone1/pid/z/setpoint', 5)
        self.state_disz_pub = self.create_publisher(Float64, '/drone1/pid/z/state', 5)

        # Call on_timer function every second
        #self.timer = self.create_timer(0.1, self.on_timer)   
      
        
    def call_back(self,msg):
        enable_angle = Bool()
        setpoint_angle = Float64()
        state_angle = Float64()

        enable_dish = Bool()
        setpoint_dish = Float64()
        state_dish = Float64()

        enable_disv = Bool()
        setpoint_disv = Float64()
        state_disv = Float64()
        
        enable_disz = Bool()
        setpoint_disz = Float64()
        state_disz = Float64()

    
        enable_angle.data = True
        state_angle.data = float(msg.angular.z)
        setpoint_angle.data = 0.0

        enable_dish.data = True
        state_dish.data = float(msg.linear.x)
        setpoint_dish.data = 0.0

        enable_disv.data = True
        state_disv.data = float(msg.linear.y)
        setpoint_disv.data = 0.0

        enable_disz.data = True
        state_disz.data = float(msg.linear.z)
        setpoint_disz.data = 0.0

        #print("sending msg now")
        #if abs(state_angle.data) <= 0.05:
        #    state_angle = 0.05  
        
        self.enable_angle_pub.publish(enable_angle)
        self.state_angle_pub.publish(state_angle)
        self.setpoint_angle_pub.publish(setpoint_angle)

        self.enable_dish_pub.publish(enable_dish)
        self.state_dish_pub.publish(state_dish)
        self.setpoint_dish_pub.publish(setpoint_dish)

        self.enable_disv_pub.publish(enable_disv)
        self.state_disv_pub.publish(state_disv)
        self.setpoint_disv_pub.publish(setpoint_disv)
        
        self.enable_disz_pub.publish(enable_disz)
        self.state_disz_pub.publish(state_disz)
        self.setpoint_disz_pub.publish(setpoint_disz)
  



def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()