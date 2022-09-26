
from time import sleep
from simplejson import simple_first
from std_srvs.srv import Empty
from matplotlib.pyplot import text
from torch import constant_pad_nd, set_flush_denormal
import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

class blind_fly_server(Node):

    def __init__(self):
        super().__init__('blind_fly_server')
        self.pub = self.create_publisher(Twist,'/drone1/cmd_vel',10)
        self.create_service(Empty,'blind_fly',self.blind_fly_callback)
        #self.create_service(Empty,'slow_rotate',self.slow_rotate_callback)
        #self.create_timer(0.1,self.timer_callback)
        self.timer_count = 0.0
        self.blind_fly_enable = False
        self.constant_speed_x = 0.2 #for sim
        self.constant_speed_z = 0.2  

        #self.constant_speed_x = 0.5
        #self.constant_speed_z = 0.5


    def blind_fly_callback(self,request,response):
        self.timer_count = 0.0
        start_time = self.get_clock().now().seconds_nanoseconds()
        start_sec = int(start_time[0])
        start_nsec = int(start_time[1])
        print("start_sec"+str(start_sec)+"start_nsec" + str(start_nsec))

        print('success1')
        self.blind_fly_enable = True

        while (self.blind_fly_enable):
            if self.blind_fly_enable:
                
                (now_sec,now_nsec)=self.get_clock().now().seconds_nanoseconds()
                dur_sec = now_sec-start_sec
                dur_nsec = now_nsec-start_nsec
                print("now_sec"+str(now_sec)+"now_nsec" + str(now_nsec))
                #print("start_sec"+str(start_sec)+"start_nsec" + str(start_nsec))
                if (dur_nsec<0):
                    dur_sec =dur_sec - 1
                    dur_nsec = dur_nsec + math.pow(10,10)

                #print("dur_nsec" + str(dur_nsec))
                if (dur_nsec==0):
                    dur_nsec_digits = 0
                else:
                    dur_nsec_digits = int(math.log10(dur_nsec))+1
                #print("dur_nsec_digits"+str(dur_nsec_digits))

                dur_nsec = float( dur_nsec / math.pow(10, dur_nsec_digits))
                #print("pwr:"+str(math.pow(10, dur_nsec_digits)))

                #print("dur_sec"+str(dur_sec))
                #print("dur_nsec"+str(dur_nsec))

                self.timer_count = float(dur_sec+dur_nsec)

                if (self.timer_count>=0.0) and (self.timer_count <2.0):
                    msg = Twist()
                    msg.linear.x = self.constant_speed_x
                    msg.linear.z = -self.constant_speed_z
                    self.pub.publish(msg)

                    #print("dur"+str(self.timer_count))
                    sleep(0.05)
                    
                elif (self.timer_count>=2.0)and(self.timer_count<3.0):
                    msg = Twist()
                    msg.linear.x = self.constant_speed_x
                    msg.linear.z = self.constant_speed_z
                    self.pub.publish(msg)
                    #self.timer_count+1
                    #print("dur"+str(self.timer_count))
                    sleep(0.05)
                elif (self.timer_count>=3.0)and(self.timer_count<7.0):
                    msg = Twist()
                    self.pub.publish(msg)
                    #self.timer_count+1
                    #print("dur"+str(self.timer_count))
                    sleep(0.05)
                else:
                    self.blind_fly_enable = False
                    print("over")
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = blind_fly_server()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
