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
from geometry_msgs.msg import PoseStamped,Point,Pose
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
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal',10)
        # Create turtle2 velocity publisher
        #self.publisher = self.create_publisher(Twist, '/drone1/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(0.8, self.on_timer)   
        
        self.src_frame = 'map'
        self.target_frame = 'gate'
        
    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        try:
            now = rclpy.time.Time()
            now = self.get_clock().now()-rclpy.time.Duration(seconds=0.20) 
            self.trans = self.tf_buffer.lookup_transform(
                self.src_frame,
                self.target_frame,
                now)
            #self.get_logger().info(str(self.trans.transform.translation.z))
            pose_tmp =PoseStamped()
            pose_tmp.header.frame_id = 'map'
            
            pose_tmp.pose.position.x = self.trans.transform.translation.x
            pose_tmp.pose.position.y = self.trans.transform.translation.y
            pose_tmp.pose.position.z = self.trans.transform.translation.z


            pose_tmp.pose.orientation.x = self.trans.transform.rotation.x
            pose_tmp.pose.orientation.y = self.trans.transform.rotation.y
            pose_tmp.pose.orientation.z = self.trans.transform.rotation.z
            pose_tmp.pose.orientation.w = self.trans.transform.rotation.w
            
            self.goal_pub.publish(pose_tmp)


        
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform from {self.src_frame} to {self.target_frame} :{ex}')
        
            return




 
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