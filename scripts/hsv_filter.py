#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge

class ImgProc(Node):
    def __init__(self):
        super().__init__("imgproc")
        self.window_capture_name = "Capture"
        self.window_detection_name = "Detection"
        self.max_value = 255
        self.max_value_H = 360//2
        self.H = {"low" : 0, "high" : self.max_value_H}
        self.S = {"low" : 0, "high" : self.max_value}
        self.V = {"low" : 0, "high" : self.max_value}

        self.H_name = ("low_H", "high_H")
        self.S_name = ("low_S", "high_S")
        self.V_name = ("low_V", "high_V")

        self.setup_window()

        self.pub = self.create_publisher(Image, "processed", 10)
        self.create_subscription(Image, '/drone1/image_raw', self.img_cb, 10)

    def img_cb(self, msg):
        self.pub.publish(msg)
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") ##
        #frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB) ##
        self.hsv_filter_slider(frame)

    def setup_window(self):
        cv.namedWindow(self.window_capture_name, cv.WINDOW_NORMAL)
        cv.namedWindow(self.window_detection_name, cv.WINDOW_NORMAL)

        cv.createTrackbar(self.H_name[0], self.window_detection_name, self.H["low"], self.max_value_H, self.on_low_H_thresh_trackbar)
        cv.createTrackbar(self.H_name[1], self.window_detection_name, self.H["high"], self.max_value_H, self.on_high_H_thresh_trackbar)
        cv.createTrackbar(self.S_name[0], self.window_detection_name, self.S["low"], self.max_value, self.on_low_S_thresh_trackbar)
        cv.createTrackbar(self.S_name[1], self.window_detection_name, self.S["high"], self.max_value, self.on_high_S_thresh_trackbar)
        cv.createTrackbar(self.V_name[0], self.window_detection_name, self.V["low"], self.max_value, self.on_low_V_thresh_trackbar)
        cv.createTrackbar(self.V_name[1], self.window_detection_name, self.V["high"], self.max_value, self.on_high_V_thresh_trackbar)

    def on_low_H_thresh_trackbar(self, val):
        self.H["low"] = val
        self.H["low"] = min(self.H["high"] - 1, self.H["low"])
        cv.setTrackbarPos(self.H_name[0], self.window_detection_name, self.H["low"])

    def on_high_H_thresh_trackbar(self, val):
        self.H["high"] = val
        self.H["high"] = max(self.H["high"], self.H["low"] + 1)
        cv.setTrackbarPos(self.H_name[1], self.window_detection_name, self.H["high"])

    def on_low_S_thresh_trackbar(self, val):
        self.S["low"] = val
        self.S["low"] = min(self.S["high"] - 1, self.S["low"])
        cv.setTrackbarPos(self.S_name[0], self.window_detection_name, self.S["low"])

    def on_high_S_thresh_trackbar(self, val):
        self.S["high"] = val
        self.S["high"] = max(self.S["high"], self.S["low"] + 1)
        cv.setTrackbarPos(self.S_name[1], self.window_detection_name, self.S["high"])

    def on_low_V_thresh_trackbar(self, val):
        self.V["low"] = val
        self.V["low"] = min(self.V["high"] - 1, self.V["low"])
        cv.setTrackbarPos(self.V_name[0], self.window_detection_name, self.V["low"])

    def on_high_V_thresh_trackbar(self, val):
        self.V["high"] = val
        self.V["high"] = max(self.V["high"], self.V["low"] + 1)
        cv.setTrackbarPos(self.V_name[1], self.window_detection_name, self.V["high"])

    def hsv_filter_slider(self, frame):
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_threshold = cv.inRange(frame_hsv, (self.H["low"], self.S["low"], self.V["low"]), (self.H["high"], self.S["high"], self.V["high"]))

        cv.imshow(self.window_capture_name, frame)
        cv.imshow(self.window_detection_name, frame_threshold)

        key = cv.waitKey(30)
        if key == ord('q') or key == 27:
            return 0

def main():
    rclpy.init()
    img_proc = ImgProc()
    try:
        rclpy.spin(img_proc)
    except KeyboardInterrupt:
        pass
    img_proc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()