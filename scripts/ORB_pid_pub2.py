#from _typeshed import Self
from ctypes import pointer
import math
from cmath import pi
from re import S
from turtle import st
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('pid_pub_2')
        # Create turtle2 velocity publisher
        
        #self.waypoint_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.call_back_goal,10)
        self.waypoint_sub = self.create_subscription(PoseStamped, '/waypoint', self.call_back_goal,10)
        self.waypoint_sub = self.create_subscription(PoseStamped, '/orb_slam2_mono/pose', self.call_back_state,10)

        
        #self.error_sub = self.create_subscription(Float64, '/drone1/pid/angle/control_effort', self.call_back_output,10)

        self.enable_angle_pub = self.create_publisher(Bool, '/drone1/pid/az/enable', 5)
        self.setpoint_angle_pub = self.create_publisher(Float64, '/drone1/pid/az/setpoint', 5)
        self.state_angle_pub = self.create_publisher(Float64, '/drone1/pid/az/state', 5)
        
        self.enable_x_pub = self.create_publisher(Bool, '/drone1/pid/x/enable', 5)
        self.setpoint_x_pub = self.create_publisher(Float64, '/drone1/pid/x/setpoint', 5)
        self.state_x_pub = self.create_publisher(Float64, '/drone1/pid/x/state', 5)

        self.enable_y_pub = self.create_publisher(Bool, '/drone1/pid/y/enable', 5)
        self.setpoint_y_pub = self.create_publisher(Float64, '/drone1/pid/y/setpoint', 5)
        self.state_y_pub = self.create_publisher(Float64, '/drone1/pid/y/state', 5)
        
        self.enable_z_pub = self.create_publisher(Bool, '/drone1/pid/z/enable', 5)
        self.setpoint_z_pub = self.create_publisher(Float64, '/drone1/pid/z/setpoint', 5)
        self.state_z_pub = self.create_publisher(Float64, '/drone1/pid/z/state', 5)




        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.src_frame = 'map'
        self.target_frame = 'camera_link'
        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)   


        self.gate_center = 0.032

        self.enable = Bool()
        self.enable.data = True

        self.setpoint_angle_yaw_z = Float64()
        self.state_angle_yaw_z = Float64()

        self.setpoint_x = Float64()
        self.state_x = Float64()

        self.setpoint_y = Float64()
        self.state_y = Float64()

        self.setpoint_z = Float64()
        self.state_z = Float64()
        
        self.setpoint_z.data = 2*self.gate_center



    def call_back_goal(self,msg):
        self.setpoint_x.data = msg.pose.position.x
        self.setpoint_y.data = msg.pose.position.y
        self.setpoint_z.data = msg.pose.position.z

        q=[0.0,0.0,0.0,0.0]
        q[0] = msg.pose.orientation.x
        q[1] = msg.pose.orientation.y
        q[2] = msg.pose.orientation.z
        q[3] = msg.pose.orientation.w
        _, _, self.setpoint_angle_yaw_z.data = self.euler_from_quaternion(q[0],q[1],q[2],q[3])
        

    def call_back_state(self,msg):
        self.state_x.data = msg.pose.position.x
        self.state_y.data = msg.pose.position.y
        self.state_z.data = msg.pose.position.z

        q=[0.0,0.0,0.0,0.0]
        q[0] = msg.pose.orientation.x
        q[1] = msg.pose.orientation.y
        q[2] = msg.pose.orientation.z
        q[3] = msg.pose.orientation.w
        _, _, self.state_angle_yaw_z.data = self.euler_from_quaternion(q[0],q[1],q[2],q[3])


    def on_timer(self):
        # transform all vel into local coordinate system : for x, y decoupled , for az, avoided step from -pi to pi

        state_angle = np.arctan2(self.state_y.data,self.state_x.data) - self.state_angle_yaw_z.data
        state_mag = np.sqrt(self.state_y.data*self.state_y.data + self.state_x.data*self.state_x.data)
        setpoint_angle = np.arctan2(self.setpoint_y.data,self.setpoint_x.data) - self.state_angle_yaw_z.data
        setpoint_mag = np.sqrt(self.setpoint_y.data*self.setpoint_y.data + self.setpoint_x.data*self.setpoint_x.data)
        
        state_x_local = Float64()
        state_y_local = Float64()

        setpoint_x_local = Float64()
        setpoint_y_local = Float64()

        state_x_local.data = state_mag * np.cos(state_angle)
        state_y_local.data = state_mag * np.sin(state_angle)
        setpoint_x_local.data = setpoint_mag * np.cos(setpoint_angle)
        setpoint_y_local.data = setpoint_mag * np.sin(setpoint_angle)

        


        self.enable_x_pub.publish(self.enable)
        self.state_x_pub.publish(state_x_local)
        self.setpoint_x_pub.publish(setpoint_x_local)

        self.enable_y_pub.publish(self.enable)
        self.state_y_pub.publish(state_y_local)
        self.setpoint_y_pub.publish(setpoint_y_local)

        self.enable_z_pub.publish(self.enable)
        self.state_z_pub.publish(self.state_z)
        self.setpoint_z_pub.publish(self.setpoint_z)




        if self.setpoint_angle_yaw_z.data * self.state_angle_yaw_z.data < 0:
            if self.setpoint_angle_yaw_z.data < -pi/2 and self.state_angle_yaw_z.data > pi/2:
                self.setpoint_angle_yaw_z.data = self.setpoint_angle_yaw_z.data + 2 * pi
            if self.setpoint_angle_yaw_z.data > pi/2 and self.state_angle_yaw_z.data < -pi/2:
                self.setpoint_angle_yaw_z.data = self.setpoint_angle_yaw_z.data - 2 * pi

                
        self.enable_angle_pub.publish(self.enable)
        self.state_angle_pub.publish(self.state_angle_yaw_z)
        self.setpoint_angle_pub.publish(self.setpoint_angle_yaw_z)




    def euler_from_quaternion(self,x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()