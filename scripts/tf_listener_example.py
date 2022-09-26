#!/usr/bin/env python3

import rclpy
import tf2_ros

def timercallback():
    global tfBuffer
    try:
        # do a lookup transform between 'base_link' and 'marker' frame
        trans = tfBuffer.lookup_transform("map", "gate", rclpy.duration.Duration())
        # returns TransformStamped() message
        print(trans.transform.translation.z) # print lookup transform
    except:
        # exception is raised on extrapolation, 
        # no connection between frames or when frames dont exist
        print("lookup failed") 

def main():
    global tfBuffer
    rclpy.init() # init ros client library
    nh = rclpy.create_node('tf2_listener') # create a node with name 'tf2_listener'
    tfBuffer = tf2_ros.Buffer() # create a TF2 buffer which saves the TFs for given cache_time
    tf2_ros.TransformListener(tfBuffer, nh) # create TF2 listener which connects buffer with node
    nh.create_timer(0.1, timercallback) # call timercallback every 100ms

    try:
        rclpy.spin(nh) # spin node until exception
    except KeyboardInterrupt:
        nh.destroy_node() # destroy node
        rclpy.shutdown() # shutdown ros client library

if __name__ == '__main__':
    main()
