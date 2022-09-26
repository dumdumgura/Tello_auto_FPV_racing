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
        self.publisher = self.create_publisher(Twist, '/drone1/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(0.2, self.on_timer)   
        
        self.src_frame = 'tello_base'
        self.target_frame = 'marker1'
        
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        try:
            now = rclpy.time.Time()
            self.trans = self.tf_buffer.lookup_transform(
                self.src_frame,
                self.target_frame,
                now)
            self.tello_center()
        
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform from {self.src_frame} to {self.target_frame} :{ex}')
            self.tello_hovering()
            return


    def tello_center(self):
        msg = Twist()
        scale_rotation_rate = 0.3   
        scale_forward_speed = 0.15
        hold_dis = 1

        #angle = math.atan2(self.trans.transform.translation.y,self.trans.transform.translation.x)
        #distance = math.sqrt(self.trans.transform.translation.x ** 2 + self.trans.transform.translation.y ** 2)

        q = [0,0,0,0]
        q[0] = self.trans.transform.rotation.x
        q[1] = self.trans.transform.rotation.y
        q[2] = self.trans.transform.rotation.z
        q[3] = self.trans.transform.rotation.w
        
        R = quaternion_rotation_matrix(q)
        self.get_logger().info(str(R[0][0])+" "+str(R[0][1])+" "+str(R[0][2]))
        self.get_logger().info(str(R[1][0])+" "+str(R[1][1])+" "+str(R[1][2]))
        self.get_logger().info(str(R[2][0])+" "+str(R[2][1])+" "+str(R[2][2]))

        #rotation =tf_transformations.euler_from_quaternion(q)
        self.get_logger().info(("z:"))
        #self.get_logger().info((str(np.round(rotation[2],decimals=2))))

        error_x = self.trans.transform.translation.x + hold_dis * R[0][2]
        error_y = self.trans.transform.translation.y + hold_dis * R[0][1]
        distance =math.sqrt(error_x*error_x+error_y*error_y)


        if distance >= 0.2:
            msg.linear.x = scale_forward_speed * error_x
            msg.linear.y = scale_forward_speed * error_y
            self.get_logger().info(('{distance},distance>0.1'))
            self.get_logger().info(("x:"+str(np.round(self.trans.transform.translation.x,decimals=2))))
            self.get_logger().info(("y:"+str(np.round(self.trans.transform.translation.y,decimals=2))))
            self.get_logger().info(("errx:"+str(np.round(error_x,decimals=2))))
            self.get_logger().info(("erry:"+str(np.round(error_y,decimals=2))))
            self.get_logger().info(("a:"+str(np.round((-np.arctan(R[0][1]/R[0][2])*180/3.1415926),decimals=2))))
        else:
            msg.linear.x = float(0.0)
            msg.linear.y = float(0.0)
            self.get_logger().info(('{distance},distance<0.1'))
        
        msg.angular.z = scale_rotation_rate * np.arctan(R[0][1]/R[0][2])
        msg.linear.z = scale_forward_speed * self.trans.transform.translation.z 
         
        self.publisher.publish(msg)
    


    def tello_hovering(self):
        msg = Twist()
        msg.angular.x = 0.0
        msg.linear.x = 0.0
        msg.angular.y = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        msg.linear.z = 0.0
         
        self.publisher.publish(msg)

 
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()