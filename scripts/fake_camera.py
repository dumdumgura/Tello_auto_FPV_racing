import rclpy
from rclpy.node import Node
import cv2 as cv
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class fake_camera(Node):

    def __init__(self):
        super().__init__('fake_camera')
        self.pub = self.create_publisher(Image, "/camera/rgb/image_raw", 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)
        print(os.getcwd())
        self.count = 1
        filefolder_path = os.path.join(os.getcwd(),'src','tello_mod', 'cv_tutorials', '0_resources')
        dogv_path = os.path.join(filefolder_path,'Two_gate.mp4')
        #out_file =os.path.join(filefolder_path,'gray_fliped.avi')
        self.cap = cv.VideoCapture(dogv_path)

    def timer_callback(self):
        #self.get_logger().info('frame:%c',self.count)
        try:
            _,cv_image = self.cap.read()
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            print("bgr now")
        except CvBridgeError as e:
           print(e)

        cv.imshow("Image window", cv_image)
        cv.waitKey(10)
        self.pub.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)

    fake_camera_pub = fake_camera()

    rclpy.spin(fake_camera_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_camera_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
