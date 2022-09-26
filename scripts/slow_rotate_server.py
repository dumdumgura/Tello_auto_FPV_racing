
from queue import Empty
from matplotlib.pyplot import text
import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class slow_rotate_server(Node):
    def __init__(self):
        super().__init__('slow_rotate_server')
        self.pub = self.create_publisher(Twist,'/drone1/cmd_vel',5)
        self.create_service(SetBool,'slow_rotate',self.rotate_callback)
        self.create_timer(0.1,self.pub_callback)
        self.rotate_enable = False
        self.rotate_status = False
        self.vel_az = -0.2 #for real
        #self.vel_az = 0.12
        self.count = 0


    def pub_callback(self):
        if self.rotate_status and self.rotate_enable:
            mymsg = Twist()
            mymsg.angular.z = -self.vel_az
            self.pub.publish(mymsg)
            print('rotating')
        
        elif self.rotate_status and (not self.rotate_enable):
            if self.count <=10:
                mymsg = Twist()
                self.pub.publish(mymsg)
                self.count = self.count + 1
                print('stopping'+str(self.count))
            else:
                self.count = 0
                self.rotate_status = False
                print('already stopped')

        pass


    def rotate_callback(self,req,res):
        self.rotate_enable = req.data
        if self.rotate_enable:
            self.rotate_status = True
        res.success = True
        return res


def main(args=None):
    rclpy.init(args=args)

    st = slow_rotate_server()
    
    
    rclpy.spin(st)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    st.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
