
from time import sleep
from matplotlib.pyplot import text
import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import _thread

class graph(Node):
    def __init__(self):
        super().__init__('graph')
        self.subscription = self.create_subscription(
            Twist,
            '/drone1/error_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.fig = plt.figure()
        
        self.azplot = self.fig.add_subplot(4,1,1)
        self.xplot = self.fig.add_subplot(4,1,2)
        self.yplot = self.fig.add_subplot(4,1,3)
        self.zplot = self.fig.add_subplot(4,1,4)

        self.azplot.title.set_text('error_y')
        self.xplot.title.set_text('error_x')
        self.yplot.title.set_text('error_az')
        self.zplot.title.set_text('error_z')

        self.erraz = []
        self.errx = []
        self.erry = []
        self.errz = []

        self.xs = []
        self.count = 0
        self.y = 0.0


    def listener_callback(self,msg):
        #print("received msg:"+str(msg.axes[0]))
        #y=np.cos(float(i)/10) + 0.3*np.cos(float(i)/15)
        self.xs.append(float(self.count))
        self.erraz.append(float(msg.linear.y))
        self.errx.append(float(msg.linear.x))
        self.erry.append(float(np.round((msg.angular.z*180/3.1415926),decimals=2)))
        self.errz.append(float(msg.linear.z))
        #print("len(x):"+str(len(self.xs)))
        
        self.azplot.plot(self.xs, self.erraz)
        self.xplot.plot(self.xs, self.errx)
        self.yplot.plot(self.xs, self.erry)
        self.zplot.plot(self.xs, self.errz)
        #print(len(self.xs))
        #print(len(self.ys))
        if len(self.xs) > 100:
            #self.yangle.xlim(self.xs[0],self.xs[100])
            self.azplot.set_xlim(self.xs[0],self.xs[100])
            self.xplot.set_xlim(self.xs[0],self.xs[100])
            self.yplot.set_xlim(self.xs[0],self.xs[100])
            self.zplot.set_xlim(self.xs[0],self.xs[100])
            
            self.xs.pop(0)
            self.erraz.pop(0)
            self.errx.pop(0)
            self.erry.pop(0)
            self.errz.pop(0)
        self.count =self.count+1
        plt.draw()
        plt.pause(0.00000001)



def main(args=None):
    rclpy.init(args=args)
    global gr
    gr = graph()
    rclpy.spin(gr)
    gr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
