
import imp
from logging import makeLogRecord
from tkinter import SCROLL
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from time import sleep
from matplotlib.pyplot import text
import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tello_msgs.srv import TelloAction
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tutorial_interfaces.srv import MarkerId   
from tutorial_interfaces.msg import BoundBox

class strategy(Node):
    def __init__(self):
        super().__init__('strategy')
        self.start_client = self.create_client(TelloAction, '/drone1/tello_action')
        self.rotate_client = self.create_client(SetBool,'/drone1/slow_rotate')
        self.forward_client = self.create_client(SetBool,'/drone1/slow_forward')

        self.approach_client = self.create_client(SetBool,'/drone1/pid/teleop_enbale')
        self.status_pub = self.create_subscription(Bool,'/drone_status',self.cb_status,10)

        self.flythrough_client = self.create_client(Empty,'/drone1/blind_fly')
        self.markerid_client = self.create_client(MarkerId,'/drone1/markerid_set')
        self.approach_flag = self.create_service(Empty,'/drone1/approach_flag',self.approach_complete_callback)
        self.boundbox = self.create_subscription(BoundBox,'/drone1/hsv_boundbox',self.hsv_callback,10)
        
        self.gate_rotate_pub = self.create_publisher(Twist,'/drone1/cmd_vel',10)
        
        #self.teleop_client = self.create_client(SetBool,'/drone1/teleop_enable')
        self.frame_width = 960
        self.frame_height = 720
        self.approach_ratio = 1.2
        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        self.status = False
        
        self.gate_rotate_vel_y = 0.05 * self.approach_ratio
        self.gate_rotate_vel_az = -0.012
        self.pd_ratio = 0.50
        self.ratio_previous = 0.0
        self.rotate_dir = -1.0 # for cw
        self.rotate_diff = 1.0#

        self.approach_flag = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def status_reset(self):
        self.status = False


    def cb_status(self,msg):
        self.status = msg.data 

    def hsv_callback(self,msg):
        self.x = msg.xmin
        self.y = msg.ymin
        self.w = msg.width
        self.h = msg.height
        #self.get_logger().info(('tolerance achieved'))
        #centerx = (2*x+w)/2
        #centery = (2*y+h)/2
        pass
    
    def hsv_center_check(self):
        ratio = float(self.h*1.0 / self.w)
        contours, hierarchy = cv.findContours(closing, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for cntt in contours:
            cv.drawContours(res, cntt, -1, (0,255,255), 3)   #yellow for all contours
            #area = cv.contourArea(cnt)
            #op = 0
        if len(contours)!=0 :
            cnt = max(contours, key = cv.contourArea)
            area = cv.contourArea(cnt) 
        
        if area > 5000:
            self.get_logger().info(('currently ratio'+str(np.round(ratio,decimals=4))))
            if abs(ratio - self.pd_ratio) < 0.005:
                return True
            else:
                return False
        else :
            return False

    def check_z_distance(self):
        centery = (2*self.y+self.h)/2
        if abs(centery-self.frame_height/2.0) <  1/10.0*self.frame_height:
            self.get_logger().info(('tolerance achieved'+str(int(abs(centery-self.frame_height/2.0)))))
            return True
        else:
            return False


    def z_adjust(self):
        centery = (2*self.y+self.h)/2
        self.z_adjust_vel =  -(centery-self.frame_height/2.0)/self.frame_height *2
        mymsg = Twist()
        mymsg.linear.z = float(self.z_adjust_vel)
        self.gate_rotate_pub.publish(mymsg)
    
    def hover(self):
        msg = Twist()
        msg.angular.x = 0.0
        msg.linear.x = 0.0
        msg.angular.y = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        msg.linear.z = 0.0
        self.gate_rotate_pub.publish(msg)

    
    def check_az_distance(self):
        centerx = (2*self.x+self.w)/2
        if abs(centerx-self.frame_width/2.0) <  1/40.0*self.frame_width:
            self.get_logger().info(('tolerance achieved'+str(int(abs(centerx-self.frame_width/2.0)))))
            return True
        else:
            return False


    def az_adjust(self):
        centerx = (2*self.x+self.w)/2
        self.az_adjust_vel =  -(centerx-self.frame_width/2.0)/self.frame_width/3.0
        mymsg = Twist()
        mymsg.angular.z = float(self.az_adjust_vel)
        self.gate_rotate_pub.publish(mymsg)


    def gate_rotate(self):
        self.ratio = float(self.h*1.0 / self.w)
        mymsg = Twist()
        mymsg.linear.x = 0.0
        mymsg.linear.z = 0.0
        # minimize
        diff = self.ratio - self.ratio_previous # first time diff>0
        if diff>0:
            self.rotate_dir = -self.rotate_dir # first time  rotate_dir = -rotate_dir = 1.0

        mymsg.linear.y = self.gate_rotate_vel_y *self.rotate_dir
        mymsg.angular.z = self.gate_rotate_vel_az *self.rotate_dir

        self.gate_rotate_pub.publish(mymsg)

        self.ratio_previous = self.ratio
        pass

    def approach_complete_callback(self,req,res):
        self.get_logger().info(('tolerance achieved'))
        self.approach_flag = True
        return res

    def check_for_approach_two(self):
        
        
        pass

    def check_for_approach_flag(self):
        if self.approach_flag:
            #self.get_logger().info(('True'))
            self.approach_flag = False
            return True
        else:
            #self.get_logger().info(('False'))
            return False

    def flag_clean(self):
        self.approach_flag = False


    def send_takeoff_request(self): 
        start_request = TelloAction.Request()
        start_request.cmd = "takeoff"
        self.start_future = self.start_client.call_async(start_request)

    def send_rotate_request(self,flag):
        req = SetBool.Request()
        req.data = flag
        self.rotate_future = self.rotate_client.call_async(req)

    def send_forward_request(self,flag):
        req = SetBool.Request()
        req.data = flag
        self.forward_future = self.forward_client.call_async(req)


    def send_approach_request(self,flag):
        req = SetBool.Request()
        req.data = flag
        self.approach_future = self.approach_client.call_async(req)
    

    def send_flythrough_request(self):
        req = Empty.Request()
        self.flythrough_future = self.flythrough_client.call_async(req)

    def send_markerid_request(self,marker):
        req = MarkerId.Request()
        req.id = marker
        self.markerid_future = self.markerid_client.call_async(req)

    #def send_teleop_request(self,flag):
    #    req = SetBool.Request()
    #    req.data = flag
    #    self.teleop_future = self.teleop_client.call_async(req)
    def hsv_check_center_horizontal(self):
        # rotate until x>1/10*frame.width, x+w<9/10*frame.width, abs(((x+w)+x)/2-frame.width)<1/10*frame.width 
        centerx = (2*self.x+self.w)/2
        centery = (2*self.y+self.h)/2
        if (self.x>1.0/10.0*self.frame_width)and((self.x+self.w)<9.0/10.0*self.frame_width)and((centerx-self.frame_width)<(1.0/10.0*self.frame_width)):
            self.get_logger().info('center testing approved')
            return True

    def hsv_check_center_height(self):
        # abs(h-desired_h)<1/20*frame.height
        if  self.h>= 3.0/5.0/self.approach_ratio*self.frame_height:
            self.get_logger().info('distance testing approved')
            return True


def wait_for_takeoff_response(st):
    while rclpy.ok():
        rclpy.spin_once(st)
        if st.start_future.done():
            try:
                response = st.start_future.result()
            except Exception as e:
                st.get_logger().info((
                    'Service call failed %r' % (e,)))
    
            break
    

def wait_rotate_response(st):
    while rclpy.ok():
        rclpy.spin_once(st)
        if st.rotate_future.done():
            try:
                response = st.rotate_future.result()
            except Exception as e:
                st.get_logger().info((
                    'Service call failed %r' % (e,)))
            break

def wait_forward_response(st):
    while rclpy.ok():
        rclpy.spin_once(st)
        if st.forward_future.done():
            try:
                response = st.forward_future.result()
            except Exception as e:
                st.get_logger().info((
                    'Service call failed %r' % (e,)))
            break


def wait_for_approach_response(st):
    while rclpy.ok():
        rclpy.spin_once(st)
        if st.approach_future.done():
            try:
                response = st.approach_future.result()
            except Exception as e:
                st.get_logger().info((
                    'Service call failed %r' % (e,)))
    
            break

def wait_for_flythrough_response(st):
    while rclpy.ok():
        rclpy.spin_once(st)
        if st.flythrough_future.done():
            try:
                response = st.flythrough_future.result()
            except Exception as e:
                st.get_logger().info((
                    'Service call failed %r' % (e,)))
            break

def wait_for_markerid_response(st):
    while rclpy.ok():
        rclpy.spin_once(st)
        if st.markerid_future.done():
            try:
                response = st.markerid_future.result()
            except Exception as e:
                st.get_logger().info((
                    'Service call failed %r' % (e,)))
            break

#def wait_for_teleop_response(st):
#    while rclpy.ok():
#        rclpy.spin_once(st)
#        if st.teleop_future.done():
#            try:
#                response = st.teleop_future.result()
#            except Exception as e:
#                st.get_logger().info((
#                   'Service call failed %r' % (e,)))
#            break



def search_for_AR_Tag(st,marker):
    find_artag1 = False
    while not find_artag1:
        try:
            now = rclpy.time.Time()
            st.trans = st.tf_buffer.lookup_transform(
                'tello_base',
                marker,
                now)
            find_artag1 = True
        except TransformException as ex:
            #st.get_logger().info(('not yet'))
            pass
        rclpy.spin_once(st)


def check_for_AR_Tag(st,marker):
    check_artag = False
    while not check_artag:
        try:
            now = rclpy.time.Time()
            st.trans = st.tf_buffer.lookup_transform(
                'tello_base',
                marker,
                now)

            check_for_artag = True



        except TransformException as ex:
            #st.get_logger().info(('not yet'))
            pass
        rclpy.spin_once(st)





def gate_artag_strategy(st,marker):


    # set marker_id for current_gate
    while not st.markerid_client.wait_for_service(timeout_sec=1.0):
        st.get_logger().info(('marker idservice not available, waiting again...'))
    st.send_markerid_request(marker)
    wait_for_markerid_response(st)
    st.get_logger().info(('currently finding Gate ..'+marker))


    # try to test ask 2 seconds, whether or not ARtag is found,  if not, rotate again to find artag 
    # until  st.check_for_approach_flag() True, break
    # y has steady error

    

    # rotate until find ARtag
    while not st.rotate_client.wait_for_service(timeout_sec=1.0):
        st.get_logger().info(('rotate service not available, waiting again...'))
    st.send_rotate_request(True)
    wait_rotate_response(st)
    st.get_logger().info(('start rotating, searching for artag now..'))


    search_for_AR_Tag(st,marker)
    st.get_logger().info(('ARtag1found, stop rotating..'))
    st.send_rotate_request(False)
    wait_rotate_response(st)
    st.get_logger().info(('successfully stopped'))



    st.get_logger().info("sofarsogood!")
    

    sleep(2)

    # approach ARtag
    st.get_logger().info("approaching ARtag!")
    while not st.approach_client.wait_for_service(timeout_sec=1.0):
        st.get_logger().info(('pid teleop service not available, waiting again...'))
    st.send_approach_request(True)
    wait_for_approach_response(st)

    while not st.check_for_approach_flag():
        rclpy.spin_once(st)
        sleep(0.5)
        #st.get_logger().info(('adjusting now...'))
    
    st.flag_clean()


    st.send_approach_request(False)
    wait_for_approach_response(st)
    st.get_logger().info("estimated approached,flythrough")
    
    sleep(5)
    #st.check_for_approach_two()

    #blind_fly_through
    
    st.get_logger().info("blind fly through ARtag!")
    while not st.flythrough_client.wait_for_service(timeout_sec=1.0):
        st.get_logger().info(('blind fly service not available, waiting again...'))
    st.send_flythrough_request()
    wait_for_flythrough_response(st)
    sleep(6)
    st.get_logger().info("already through ARtag!")
    pass






def hsv_gate_strategy(st):
    # stage 1: find the gate
    # rotate until x>1/10*frame.width, y+h<9/10*frame.width, abs(((x+w)+x)/2-frame.width)<1/10*frame.width 
    # call rotate service(SetBool), check for cv_pub in strategy
    
    st.get_logger().info(('try to find gate with hsv filter...'))

    while not st.rotate_client.wait_for_service(timeout_sec=1.0):
        st.get_logger().info(('rotate service not available, waiting again...'))
    st.send_rotate_request(True)
    wait_rotate_response(st)
    st.get_logger().info(('start rotating, searching for gate using hsv filter now..'))

    flag = False
    rclpy.spin_once(st)

    flag = st.hsv_check_center_horizontal()

    if st.hsv_check_center_horizontal():
        flag = True
        pass

    while not flag:
        rclpy.spin_once(st)
        flag = st.hsv_check_center_horizontal()
        pass
    

    st.get_logger().info(('Gate found, stop rotating..'))
    st.status_reset()
    st.send_rotate_request(False)
    wait_rotate_response(st)
    st.get_logger().info(('successfully stopped'))
    


    sleep(3)
    

    while not st.check_z_distance():
        rclpy.spin_once(st)
        st.z_adjust()
        st.get_logger().info(('z axis_adjusting..'))
        sleep(0.1)
    
    sleep(3)
 
    # keep height by making abs(((y+h)+y)/2 - frame.height)<1/10*frame.height
    # pid control, service disable tf_pub, enable pid_teleop


    # stage 2: approach the gate
    st.get_logger().info("approaching ARtag!")
    while not st.approach_client.wait_for_service(timeout_sec=1.0):
        st.get_logger().info(('pid teleop service not available, waiting again...'))
    st.send_approach_request(True)
    wait_for_approach_response(st)

    while(st.status==False):
        rclpy.spin_once(st)

    st.get_logger().info(('successfully through gate'))

    sleep(10)
    st.send_approach_request(False)


    # stage 2: approach the gate
    # slowly approch the gate until abs(h-desired_h)<1/20*frame.height
    #while not st.forward_client.wait_for_service(timeout_sec=1.0):
    #    st.get_logger().info(('forward service not available, waiting again...'))
    #st.send_forward_request(True)
    #wait_forward_response(st)
    #st.get_logger().info(('start forwarding, searching for gate using hsv filter now..'))

    #flag = False
    #rclpy.spin_once(st)

    #flag = st.hsv_check_center_height()

    #if st.hsv_check_center_height():
    #    flag = True
    #    pass

    #while not flag:
    #    rclpy.spin_once(st)
    #    flag = st.hsv_check_center_height()
    #    pass
    
    ##while not st.forward_client.wait_for_service(timeout_sec=1.0):
    #    st.get_logger().info(('forward service not available, waiting again...'))
    #st.get_logger().info(('Gate approached, stop forwarding..'))
    #st.send_forward_request(False)
    #wait_forward_response(st)
    #st.get_logger().info(('successfully stopped'))

    

    ### az adjust
    #while not st.check_az_distance():
    #    rclpy.spin_once(st)
    #    st.az_adjust()
    #    #st.get_logger().info(('z axis_adjusting..'))
    #    sleep(0.1)
    
    #st.get_logger().info(('az successfully adjusted'))

    ## z adjust
    
    
    #st.get_logger().info(('z successfully adjusted 2'))

    #hoveriing
    #count = 0 
    #while count<200:
    #    count=count+1
    #    st.hover()
    #    rclpy.spin_once(st)
    

    #st.get_logger().info(('sofarsogood3'))


     ### az adjust
    #while not st.check_az_distance():
    #    rclpy.spin_once(st)
    #    st.az_adjust()
    #    #st.get_logger().info(('z axis_adjusting..'))
    #    sleep(0.1)
    
    #st.get_logger().info(('az successfully adjusted'))

    #hoveriing
    #count = 0 
    #while count<200:
    ##    count=count+1
     #   st.hover()
     #   rclpy.spin_once(st)


    # stage 3: center to the gate
    # rotate around the gate until abs(ratio-desired_ratio)<1/20*frame.height
    # strategy give out vel?

    
    #while not st.hsv_center_check():
    #    st.gate_rotate()
    #    rclpy.spin_once(st)
    #    sleep(1/5)
    #    st.get_logger().info(('rotating around the gate now'))
    
    #hoveriing
   # count = 0 
   # while count<200:
    #    count=count+1
    #    st.hover()
    #    rclpy.spin_once(st)
    #st.get_logger().info(('center the gate successfully'))





    # stage 4: blindly fly through it
    #while not st.forward_client.wait_for_service(timeout_sec=1.0):
    #    st.get_logger().info(('forward service not available, waiting again...'))
    #st.send_forward_request(True)
    #wait_forward_response(st)
    #st.get_logger().info(('start forwarding, searching for gate using hsv filter now..'))
    # done!
    #pass

    #sleep(20)

    #while not st.forward_client.wait_for_service(timeout_sec=1.0):
    #    st.get_logger().info(('forward service not available, waiting again...'))
    #st.get_logger().info(('Gate approached, stop forwarding..'))
    #st.send_forward_request(False)
    #wait_forward_response(st)
    #st.get_logger().info(('successfully stopped'))






def main(args=None):
    rclpy.init(args=args)    
    st = strategy()
    st.get_logger().info(("preparing111"))
    sleep(5)


    #disable joy_stick_teleop
    #while not st.teleop_client.wait_for_service(timeout_sec=1.0):  
    #    st.get_logger().info(('teleop service not available, waiting again...'))
    #st.send_teleop_request(False)
    #wait_for_teleop_response(st)
    #st.get_logger().info(('teleop disabled'))
   
    # take off
    
    while not st.start_client.wait_for_service(timeout_sec=1.0):  
        st.get_logger().info(('take off service not available, waiting again...'))
    st.send_takeoff_request()
    wait_for_takeoff_response(st)
    st.get_logger().info(('successfully taken off'))
    
    sleep(5)

    #gate_artag_strategy(st,'marker1')

    #st.get_logger().info("sofarsogood1!")
  
    #gate_artag_strategy(st,'marker2')

    #gate_artag_strategy(st,'marker3')

    #st.get_logger().info("sofarsogood2!")
    while(True):
        hsv_gate_strategy(st)
    
    # logic + 
    # if I lost the view of ARtag, I have to rotate to find it again


    #disable joy_stick_teleop
    #while not st.teleop_client.wait_for_service(timeout_sec=1.0):  
    #    st.get_logger().info(('service not available, waiting again...'))
    #st.send_teleop_request(True)
    #wait_for_teleop_response(st)


    rclpy.spin(st)

    st.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
