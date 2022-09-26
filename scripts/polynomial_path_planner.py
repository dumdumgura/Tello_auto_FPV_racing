#from _typeshed import Self
import math
from cmath import pi
from re import S
from sre_parse import State
import rclpy
from rclpy.node import Node
from turtle import st
from geometry_msgs.msg import PoseStamped,Point,Pose
from nav_msgs.msg import Path
from rclpy.node import Node
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Bool


class path_planner(Node):

    def __init__(self):
        super().__init__('path_planning_2')
        # Create turtle2 velocity publisher
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.call_back_goal,10)
        self.pose_sub = self.create_subscription(PoseStamped, '/orb_slam2_mono/pose', self.call_back_state,10)
        self.status_pub = self.create_publisher(Bool,'/drone_status',10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/waypoint', 10)
        self.path_pub =self.create_publisher(Path,'planning_path',5)
        self.timer = self.create_timer(0.05,self.on_timer)
        #self.timer_two = self.create_timer(0.2,self.tf_timer)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state=[0.0,0.0,0.0]
        self.setpoint=[0.0,0.0,0.0]
        self.state_yaw = 0.0
        self.setpoint_yaw = 0.0
        self.N = 5
        self.xt = [0]
        self.yt = [0]
        self.count=1
        self.dis_tolerance = 0.10
        self.pathMsg = Path() 
        
        self.gate_center = 0.032

    def tf_timer(self):
        try:
            # do a lookup transform between 'base_link' and 'marker' frame
            
            trans = self.tf_buffer.lookup_transform("map", "gate", rclpy.duration.Duration())
            # returns TransformStamped() message
            #print(trans.transform.translation.z) # print lookup transform
            self.get_logger().info(trans.transform.translation.z)
            self.gate_center = trans.transform.translation.z#

        except:
            # exception is raised on extrapolation, 
            # no connection between frames or when frames dont exist
            self.get_logger().info("lookup failed")
            #print("lookup failed") 


    def call_back_goal(self,msg):
        self.setpoint = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
        self.gate_center = self.setpoint[2]-0.01
        q=[0.0,0.0,0.0,0.0]
        q[0] = msg.pose.orientation.x
        q[1] = msg.pose.orientation.y
        q[2] = msg.pose.orientation.z
        q[3] = msg.pose.orientation.w
        _, _, self.setpoint_yaw = self.euler_from_quaternion(q[0],q[1],q[2],q[3])
        
        self.count = 1
        self.xt,self.yt = self.planner()

        self.pathMsg = Path() 
        self.pathMsg.header.frame_id = 'map'
        now = rclpy.time.Time()
        #pathMsg.header.stamp = now

    

        self.dis_tolerance = np.sqrt((self.setpoint[0]-self.state[0])*(self.setpoint[0]-self.state[0])+
                                        (self.setpoint[1]-self.state[1])*(self.setpoint[1]-self.state[1])) / self.N / 3 * 2


        for i in range (0,self.N):
            pose_tmp =PoseStamped()
            pose_tmp.pose.position.x = self.xt[i]
            pose_tmp.pose.position.y = self.yt[i]    
            #if i <= self.N-2:
            #    pose_tmp.pose.position.z =  2 * self.gate_center
            #else:
            pose_tmp.pose.position.z =  self.state[2] - (self.state[2] - self.gate_center) * i/self.N
            # for fpv movement
            #yaw_tmp = np.arctan2((self.yt[i+1]-self.yt[i]),(self.xt[i+1]-self.xt[i]))

            # for yaw movement
            yaw_tmp = np.arctan2((self.setpoint[1]-self.yt[i]),(self.setpoint[0]-self.xt[i]))


            #yaw_lim_a = self.state_yaw

            #if yaw_lim_a <= 0 and yaw_lim_a >= -pi:
            #    yaw_lim_b = yaw_lim_a + pi
            #    if not (self.setpoint_yaw >= yaw_lim_a and self.setpoint_yaw <= yaw_lim_b):
            #        if self.setpoint_yaw > 0:
            #            yaw_tmp = yaw_lim_a + ((self.setpoint_yaw-pi*2) - yaw_lim_a) * i / self.N
            #            if yaw_tmp < -pi:
            #                yaw_tmp = yaw_tmp + 2 * pi
            #        else:
            #            yaw_tmp = yaw_lim_a + ((self.setpoint_yaw) - yaw_lim_a) * i / self.N

            #    else:
            #        yaw_tmp = yaw_lim_a + (self.setpoint_yaw - yaw_lim_a) * i / self.N
            #
            #else:
            #    yaw_lim_b = yaw_lim_a - pi
            #    if not (self.setpoint_yaw <= yaw_lim_a and self.setpoint_yaw >= yaw_lim_b):
            #        if self.setpoint_yaw < 0:
            #            yaw_tmp = yaw_lim_a + ((self.setpoint_yaw+pi*2) - yaw_lim_a) * i / self.N
            #            if yaw_tmp > pi:
            #                yaw_tmp = yaw_tmp - 2 * pi
            #        else:
            #            yaw_tmp = yaw_lim_a + ((self.setpoint_yaw) - yaw_lim_a) * i / self.N
            #
            #    else:
            #        yaw_tmp = yaw_lim_a + (self.setpoint_yaw - yaw_lim_a) * i / self.N

            

            qx,qy,qz,qw = self.euler_to_quaternion(yaw_tmp,0,0)

            pose_tmp.pose.orientation.x = qx
            pose_tmp.pose.orientation.y = qy
            pose_tmp.pose.orientation.z = qz
            pose_tmp.pose.orientation.w = qw
            
            self.pathMsg.poses.append(pose_tmp)
        

        for t in range (0,3):
            pose_tmp =PoseStamped()

            #yaw_tmp = np.arctan2((self.yt[self.N]-self.yt[self.N-1]),(self.xt[self.N-1]-self.xt[self.N-2]))
            qx,qy,qz,qw = self.euler_to_quaternion(self.setpoint_yaw,0,0)

            pose_tmp.pose.position.x = self.xt[self.N] + 0.12* t * np.cos(self.setpoint_yaw)
            pose_tmp.pose.position.y = self.yt[self.N] + 0.12* t * np.sin(self.setpoint_yaw)   
            pose_tmp.pose.position.z =  self.gate_center 

            self.xt = np.append(self.xt,pose_tmp.pose.position.x)
            self.yt = np.append(self.yt,pose_tmp.pose.position.y)

            pose_tmp.pose.orientation.x = qx
            pose_tmp.pose.orientation.y = qy
            pose_tmp.pose.orientation.z = qz
            pose_tmp.pose.orientation.w = qw

            self.pathMsg.poses.append(pose_tmp)


        self.path_pub.publish(self.pathMsg)



    def call_back_state(self,msg):
        self.state = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]

        q=[0.0,0.0,0.0,0.0]
        q[0] = msg.pose.orientation.x
        q[1] = msg.pose.orientation.y
        q[2] = msg.pose.orientation.z
        q[3] = msg.pose.orientation.w
        _, _, self.state_yaw = self.euler_from_quaternion(q[0],q[1],q[2],q[3])
            

    def on_timer(self):
        if (self.count+1)<len(self.pathMsg.poses):
            dis = np.sqrt((self.xt[self.count]-self.state[0])*(self.xt[self.count]-self.state[0])+
                                        (self.yt[self.count]-self.state[1])*(self.yt[self.count]-self.state[1]))

            #self.get_logger().info('count='+str(self.count)+'dis = '+str(dis))
            if dis<= self.dis_tolerance:
                self.count=self.count+1
                if self.count >= ( self.N+2 ):
                    msg = Bool()
                    msg.data = True
                    self.status_pub.publish(msg)
                    self.get_logger().info('goal achieved')
                    pass
                msg = PoseStamped()
                msg = self.pathMsg.poses[self.count]
                self.waypoint_pub.publish(msg)        
        #self.tf_timer()


    def euler_from_quaternion(self,x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians


    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def planner(self):
        # 2d trajectory planning

        # state a start pose (pos_s, yaw_s)
        pos_s = [self.state[0],self.state[1]]
        yaw_s = self.state_yaw /pi * 180

        #       a goal  pose (pos_g, yaw_g)
        pos_g = [self.setpoint[0],self.setpoint[1]]
        yaw_g = self.setpoint_yaw /pi * 180

        # polynomial interpolation (3x)
        # 
        # x(t)  = [a0  a1  a2  a3] * [1 t^1 t^2 t^3].T       y(t) = ...
        # dx(t) = [0   a1  2a2 3a3]* [0 1   t^1 t^2].T

        # boundary condition
        # x(0) = pos_s.x  dx(0) = cos(yaw_s/180*pi)
        # x(1) = pos_g.x  dx(1) = cos(yaw_g/180*pi)
        # y(0) = pos_s.y  dx(0) = sin(yaw_s/180*pi)
        # y(1) = pos_g.y  dx(1) = sin(yaw_g/180*pi)

        # matrix formulation 
        # for x(t)   
        # pos_s.x = a0
        # cos(yaw_s/180*pi) = a1
        # pos_g.x = a0+a1+a2+a3 
        # cos(yaw_g/180*pi) = a1 + 2a2+3a3 
        N=self.N

        T=np.matrix([[1,0,0,0],
                    [0,1,0,0],
                    [1,1,1,1],
                    [0,1,2,3]])

        k = np.sqrt((pos_s[0]-pos_g[0])*(pos_s[0]-pos_g[0])+((pos_s[1]-pos_g[1])*(pos_s[1]-pos_g[1])))
        Ta=[pos_s[0],0.2*k*np.cos(yaw_s/180*pi),pos_g[0],2.5*k*np.cos(yaw_g/180*pi)]
        Tb=[pos_s[1],0.2*k*np.sin(yaw_s/180*pi),pos_g[1],2.5*k*np.sin(yaw_g/180*pi)]

        a=np.dot(np.linalg.inv(T),np.transpose(Ta))
        b=np.dot(np.linalg.inv(T),np.transpose(Tb))


        xt = []
        yt = []
        for t in np.linspace(0, 1, N+1, endpoint=True):
            xt_tmp = float(np.dot(a,[1,t,t*t,t*t*t]))
            yt_tmp = float(np.dot(b,[1,t,t*t,t*t*t]))
            xt = np.append(xt,xt_tmp)
            yt = np.append(yt,yt_tmp)

        # t = np.linspace(0, 1, N, endpoint=True)

        return xt,yt

def main():
    rclpy.init()
    node = path_planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()