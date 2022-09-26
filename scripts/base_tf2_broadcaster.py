from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
import tf_transformations

class FixedFrameBroadcaster(Node):

   def __init__(self):
      super().__init__('fixed_frame_tf2_broadcaster')
      self.br = TransformBroadcaster(self)
      self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
      t = TransformStamped()
      t.header.stamp = self.get_clock().now().to_msg()
      #print(t.header.stamp.sec)
      
      t.header.frame_id = 'camera_link' # for simulation
      
      #t.header.frame_id = 'camera_frame'  # for real

      t.child_frame_id = 'optical'
   
      # Rotate the previous pose by 180* about X
   
      t.transform.translation.x = 0.0
      t.transform.translation.y = 0.0
      t.transform.translation.z = 0.0
    
      q_orig = tf_transformations.quaternion_from_euler(0, 0, 0)
      # rotate around x axis +90 deg
      q_rot1 = tf_transformations.quaternion_from_euler(0, 3.14159/2, 0)
      
      # rotate around z axis -90 deg
      q_rot2 = tf_transformations.quaternion_from_euler(-3.14159/2, 0, 0)
      

      q_new1 = tf_transformations.quaternion_multiply(q_rot1, q_orig)
      q_new2 = tf_transformations.quaternion_multiply(q_rot2, q_new1)
      
      t.transform.rotation.x = q_new2[0]
      t.transform.rotation.y = q_new2[1]
      t.transform.rotation.z = q_new2[2]
      t.transform.rotation.w = q_new2[3]

      self.br.sendTransform(t)

      


def main():
   rclpy.init()
   node = FixedFrameBroadcaster()
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()