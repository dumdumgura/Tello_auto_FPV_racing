
from matplotlib.pyplot import text
import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.subscription  # prevent unused variable warning
        self.i = 0

    def listener_callback(self, msg):
        self.get_logger().info('image received')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        except CvBridgeError as e:
           print(e)
        
        #cv.imshow('gaus',cv_image)
        #print(self.i)
        #if self.i%10 ==0:
        #    cv.imwrite('/home/hongli/tello_ws/src/tello_mod/tello_mod/pic/zo'+str(self.i)+'.jpg',cv_image)
        
        
        #self.i=self.i+1
            
        
        #cv.destroyAllWindows()
        cv.imshow("Image window", cv_image)
        #cv.waitKey(3)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
