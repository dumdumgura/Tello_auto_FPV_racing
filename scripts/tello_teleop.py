#!/usr/bin/env python3
from posixpath import lexists
import numpy as np
import rclpy
from rclpy import node
from rosidl_parser.definition import FLOATING_POINT_TYPES
from std_msgs import msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tello_msgs.srv import TelloAction
import math
from std_srvs.srv import SetBool


def main():
    global mypub
    global start_service
    global start_request
    global node_handle
    global takeoff_former
    global enable

    enable = False

    takeoff_former = False
    global land_former
    land_former = False
    rclpy.init()
    node_handle = rclpy.create_node('teleop')
    mysub  = node_handle.create_subscription(Joy,'/drone1/joy',teleop_callback,1)
    #enable_sub = node_handle.create_subscription(Bool,'/joy_enable',enable_callback,10)
    
    mypub = node_handle.create_publisher(Twist, '/drone1/cmd_vel', 1)
    start_service = node_handle.create_client(TelloAction, '/drone1/tello_action')

    enable_service= node_handle.create_service(SetBool,'/drone1/teleop_enbale',enable_callback)

    #print("1")
    #while not start_service.wait_for_service(timeout_sec=1.0):
    #    print('service not available, trying again...')

    #print("2")

    try:
        rclpy.spin(node_handle)
    except KeyboardInterrupt:
        pass

    node_handle.destroy_node()
    rclpy.shutdown()

def enable_callback(request, response):
    global enable
    enable = request.data
    return response



def teleop_callback(msg):
    global takeoff_former
    global land_former

    takeoff = bool(msg.buttons[0])
    land = bool(msg.buttons[1])

    safe_coef = 0.5

    if(takeoff and (not takeoff_former)):  
        start_request = TelloAction.Request()
        start_request.cmd = "takeoff"
        start_future = start_service.call_async(start_request)
        takeoff_flag = False
        print("already sent takeoff")
    else:
        if(land and (not land_former)):
            start_request = TelloAction.Request()
            start_request.cmd = "land"
            start_service.call_async(start_request)
            print("already sent land")

        else:
            lx = float(safe_coef*msg.axes[1]) 
            ly = float(safe_coef*msg.axes[0])
            lz = float(safe_coef*msg.axes[4])
            rz = float(safe_coef*msg.axes[3])
            
            twist_callback(lx,ly,lz,rz)
    
    takeoff_former = takeoff
    land_former = land
    
    #theta = math.atan2(x,y) 
    #mag = x*x + y*y
    


def twist_callback(lx,ly,lz,rz):
    global mypub
    mymsg = Twist()
    mymsg.linear.x = lx
    mymsg.linear.y = ly
    mymsg.linear.z = lz
    mymsg.angular.z = rz
    #mymsg.linear = [[1], [2] , [3]]
    mypub.publish(mymsg)

if __name__ == '__main__':
    main()