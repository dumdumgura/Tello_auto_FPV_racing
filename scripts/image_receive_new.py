from cmath import pi
from email.mime import image
#from pyexpat import model
#from tkinter import CENTER
#from typing_extensions import get_overloads
#from matplotlib.pyplot import text
import rclpy
from rclpy.node import Node
import cv2 as cv
from tutorial_interfaces.msg import BoundBox
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import PoseStamped,Point,Pose

import tf_transformations 
from tf2_ros import TransformBroadcaster
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('image_receive')
        self.subscription = self.create_subscription(Image,'/drone1/image_raw',self.listener_callback,10)
        self.pub = self.create_publisher(BoundBox,'/drone1/hsv_boundbox',10)
        self.hsv_pub = self.create_publisher(Image,'/drone1/hsv',10)
        self.pose_sub = self.create_subscription(PoseStamped, '/orb_slam2_mono/pose', self.call_back_state,10)
        #self.darknet_ros_sub = self.create_subscription(BoundingBoxes,'/darknet_ros/bounding_boxes',self.darknet_ros_callback,10)
        self.bridge = CvBridge()
        self.state_yaw = 0
        self.pub_pnp = self.create_publisher(PoseStamped, '/pnpPose', 1)
        self.subscription  # prevent unused variable warning
        self.Arrtsl = np.array([[0,0,0]],dtype="double")
        self.Arrp = np.array([[0]],dtype="double")
        self.br = TransformBroadcaster(self)




        x_p = [-0.7,  -0.38 , 0.0 , 0.38, 0.7]  
        x_p_r = list(np.asarray(x_p)*200 + 140)

        y_p = [-0.34, 0.17,  0.34,  0.17,-0.34] 
        y_p_r = list(np.asarray(y_p)*200 - 651)

        self.coeff = np.polyfit(x_p_r,y_p_r,4)
        xt = np.linspace(0,280,21)
        yt = -(self.coeff[0]*xt*xt*xt*xt + self.coeff[1]*xt*xt*xt + self.coeff[2]*xt*xt + self.coeff[3]*xt + self.coeff[4])
        
        self.ptr = np.array([[0,0]])
        ptr_n = np.array([[0,0]])
        for i in range(len(xt)):
            self.ptr = np.append(self.ptr,[[xt[i],yt[i]]],axis=0)
            #ptr_n = np.append(ptr_n,[[x[i],y[i]]],axis=0)
        self.ptr = np.delete(self.ptr,0,0)


        ipt_A = [0,720-1]
        ipt_B = [0,583]
        ipt_C = [280,583]
        ipt_D = [280,720-1]
        self.ipts =np.float32([ipt_A,ipt_B,ipt_C,ipt_D])

        coeff = np.array([0.34, 0, -1.09, 0, -0.609],dtype="double")
        self.model_points = np.array([[0,0,0]],dtype="double")

        i=0
        while i<=20:
            xt = -0.7 + i * 1.4/20
            #print("xt="+str(xt))
            xt_c = np.array([1, xt, xt*xt, xt*xt*xt, xt*xt*xt*xt],dtype="double")
            yt = np.inner(coeff, xt_c)
            #print("yt="+str(yt))
            #print(yt)
            tmp = np.array([0,yt,xt],dtype="double")
            tmp_2 = np.array([0.45,yt,xt],dtype="double")
            self.model_points =np.append(self.model_points,[tmp],axis=0)
            self.model_points =np.append(self.model_points,[tmp_2],axis=0)
            i=i+1

        self.model_points = np.delete(self.model_points,0,0)


    def darknet_ros_callback(self,msg):
        print("success")
    
    def call_back_state(self,msg):
        # self.state = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
        q=[0.0,0.0,0.0,0.0]
        q[0] = msg.pose.orientation.x
        q[1] = msg.pose.orientation.y
        q[2] = msg.pose.orientation.z
        q[3] = msg.pose.orientation.w
        _, _, self.state_yaw = self.euler_from_quaternion(q[0],q[1],q[2],q[3])


    def listener_callback(self, msg):
        #self.get_logger().info('image received')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        except CvBridgeError as e:
           print(e)
        
        # basic image processing:

        #cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
        blur = cv.blur(cv_image,(10,10))
        #blur = cv.GaussianBlur(cv_image,(5,5),0)
        #blur = cv.bilateralFilter(cv_image,9,75,75)
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_hue = np.array([170,150,50])
        upper_hue = np.array([180,255,255])
        mask1 = cv.inRange(hsv, lower_hue, upper_hue)
        # Threshold the HSV image to get only blue colors
        
        lower_blue = np.array([0,150,50])
        upper_blue = np.array([5,255,255])
        mask2 = cv.inRange(hsv, lower_blue, upper_blue)

    
        mask = mask1+mask2
        kernel = np.ones((3,3),np.uint8)

        dilation = cv.dilate(mask,kernel,iterations = 1)
        opening = cv.morphologyEx(dilation, cv.MORPH_OPEN, kernel)
        #dilation = cv.dilate(opening,kernel,iterations = 2)
        closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
        # Bitwise-AND mask and original image
        #cv.imshow('img',closing)
        #key = cv.waitKey(30)
        res = cv.bitwise_and(cv_image,cv_image, mask = mask)
        


        #extracing contours
        contours, hierarchy = cv.findContours(closing, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for cntt in contours:
            cv.drawContours(res, cntt, -1, (0,255,255), 3)   #yellow for all contours
            #area = cv.contourArea(cnt)
            #op = 0


        box_num = 0
        area = 0
        ptr_left = np.array([0,0],dtype="double")
        ptr_right = np.array([0,0],dtype="double")


        #camera intrinsics:
        camera_matrix = np.array(
                            [[924.873180, 0.000000, 486.997346 ],[0.000000, 923.504522, 364.308527],[ 0.000000, 0.000000, 1.000000]])
   
        dist_coeffs =np.array([-0.034749, 0.071514, 0.000363, 0.003131, 0.000000])
    

        if len(contours)!=0 :
            cnt = max(contours, key = cv.contourArea)
            area = cv.contourArea(cnt)

        if area > 5000:
            box_num=1
            cv.drawContours(res, cnt, -1, (0,255,0), 3)  #green lines for enough big contours
        
            epsilon = 0.005*cv.arcLength(cnt,True)
            approx = cv.approxPolyDP(cnt,epsilon,True)
            

            flag = False
            x,y,w,h = cv.boundingRect(approx)  
            if x<= 20 or x+w >= 940 or y<=20 or y+h >=720:
                print("out of boundary")
                flag = True 


            tmp = approx[0]
            tmp_l = tmp[0]
            tmp_r = tmp[0]
            tmp_t = tmp[0]
            tmp_b = tmp[0]

            for ptr in approx:
                if ptr[0][0] < tmp_l[0]:
                    tmp_l = ptr[0]
                if ptr[0][0] > tmp_r[0]:
                    tmp_r = ptr[0]
                if ptr[0][1] < tmp_t[1]:
                    tmp_t = ptr[0]
                if ptr[0][1] > tmp_b[1]:
                    tmp_b = ptr[0]
            

            q = 0
            cnttt=0
            tmpp =0
            while q < w:
                if closing[y+1][x+q] != 0:
                    tmpp = tmpp +x+q
                    cnttt =cnttt+1
                q = q+1
            tmp_t[0]= int(tmpp/cnttt)
            tmp_t[1]= y+2


            cv.circle(res,(tmp_l[0],tmp_l[1]), 10, (0,0,255), -1)
            cv.circle(res,(tmp_r[0],tmp_r[1]), 10, (0,0,255), -1)
            cv.circle(res,(tmp_t[0],tmp_t[1]), 10, (0,0,255), -1)
            cv.circle(res,(tmp_b[0],tmp_b[1]), 10, (0,0,255), -1)


            

            l2b = (tmp_l[0]-tmp_b[0])*(tmp_l[0]-tmp_b[0])+(tmp_l[1]-tmp_b[1])*(tmp_l[1]-tmp_b[1])
            r2b = (tmp_r[0]-tmp_b[0])*(tmp_r[0]-tmp_b[0])+(tmp_r[1]-tmp_b[1])*(tmp_r[1]-tmp_b[1])
            gate_pose = 0

            if (l2b<=r2b): # gate is toward right
                gate_pose = 2
                cv.line(res,(tmp_b[0],tmp_b[1]),(tmp_r[0],tmp_r[1]),(25,123,0),10)
                cv.line(res,(tmp_b[0],tmp_b[1]),(tmp_t[0],tmp_t[1]),(25,123,0),10)
                cv.line(res,(tmp_r[0],tmp_r[1]),(tmp_t[0],tmp_t[1]),(25,123,0),10)
                ptr_left = tmp_b
                ptr_right = tmp_r  
                angle = np.arctan2( y+h - ptr_right[1], ptr_right[0]-x )     

            else:
                gate_pose = 1
                cv.line(res,(tmp_b[0],tmp_b[1]),(tmp_l[0],tmp_l[1]),(25,123,0),10)
                cv.line(res,(tmp_b[0],tmp_b[1]),(tmp_t[0],tmp_t[1]),(25,123,0),10)
                cv.line(res,(tmp_l[0],tmp_l[1]),(tmp_t[0],tmp_t[1]),(25,123,0),10)
                ptr_right = tmp_b
                ptr_left = tmp_l

            
                angle =- np.arctan2( y+h - ptr_left[1], x+w-ptr_left[0] )   
            
            print("angle="+str((angle*180/pi)))
            angle = angle*180/pi
            # print(x,y,w,h)
            #print(np.shape(res))
            cv.rectangle(res,(x,y),(x+w,y+h),(255,0,25),2)        #blue for bounding box
            
            msg  = BoundBox()
            msg.xmin = x
            msg.ymin = y
            msg.width = w
            msg.height = h
            self.pub.publish(msg)

            centerx = int(hsv.shape[1] / 2) 
            centery = int(hsv.shape[0] / 2 )
            cv.line(res,(centerx-20,centery),(centerx+20,centery),(25,123,0),3)
            cv.line(res,(centerx,centery-10),(centerx,centery+10),(25,123,0),3)

            centerbx =int((x*2+w)/2)
            centerby =int((y*2+h)/2)
            cv.line(res,(centerbx-20,centerby),(centerbx+20,centerby),(125,123,0),3)
            cv.line(res,(centerbx,centerby-10),(centerbx,centerby+10),(125,123,0),3)

            font = cv.FONT_HERSHEY_SIMPLEX
            ss = "width: " + str(w) + " height:" + str(h)
            ratio = h/w
            ss2 = "ratio: " + str(ratio)
            #Fy = 919
            #e_dis = Fy * 500/op_h
            text = str(ss)
            text2 = str(ss2)
            #cv.putText(res,text,(x,y+h), font, 1,(255,255,255),4,cv.LINE_AA)
            #cv.putText(res,text2,(x,y+h+30), font, 1,(255,255,255),4,cv.LINE_AA)
            
            i = 1
            seg=60
            dw = ((ptr_right[0]-ptr_left[0]) *1.0/ seg)
            #print(x,y,w,h)
            #ROI = closing[x:x+w][y:y+h]
            ptrr = np.array([[ptr_left[0],ptr_left[1]]],dtype="int")
            end_pt = np.array([[ptr_right[0],ptr_right[1]]],dtype="int")
                

            image_points =[]
            x_err =0
            y_err =0
            # gate towards left
            if angle<-3:
                x_err = ptr_right[0] - tmp_r[0]
                y_err = ptr_right[1] - tmp_r[1]

                
                if w>20 and h>20 and x+w<959 and y+h<719 and x>0 and y>0:
                    while i < seg:
                        q = 0
                        tmpp = 0
                        if (ptr_left[0]+int(i*dw-1))<=tmp_t[0]-20:
                            while q < h:
                                if closing[y+q][ptr_left[0]+int(i*dw-1)] != 0:
                                    tmpp = y+q
                                    break
                                q = q+1
                        
                        else:
                            if (ptr_left[0]+int(i*dw-1))>=tmp_t[0]+20: 
                                while q < h:
                                    if closing[y+q][ptr_left[0]+int(i*dw-1)] != 0:
                                        tmpp = y+q
                                    q = q+1
                            
                            else:
                                i = i+1  
                                continue

                        ptrr_tmp = np.array([ptr_left[0]+int(i*dw-1),tmpp],dtype="double")
                        #print(ptrr_tmp)
                        ptrr = np.append(ptrr,[ptrr_tmp],axis=0)
                        i = i+1  

        

            # gate towards right
            if angle>3:
                x_err = ptr_left[0] - tmp_l[0]
                y_err = ptr_left[1] - tmp_l[1]
                
                if w>20 and h>20 and x+w<960 and y+h<720:
                    while i < seg:
                        q = 0
                        tmpp = 0
                        if (ptr_left[0]+int(i*dw-1))>=tmp_t[0]+40:
                            while q < h:
                                if closing[y+q][ptr_left[0]+int(i*dw-1)] != 0:
                                    tmpp = y+q
                                    break
                                q = q+1
                        else:
                            if (ptr_left[0]+int(i*dw-1))<=tmp_t[0]-40:
                                while q < h:
                                    if closing[y+q][ptr_left[0]+int(i*dw-1)] != 0:
                                        tmpp = y+q
                                    q = q+1
                            else:
                                i = i+1  
                                continue
                        ptrr_tmp = np.array([ptr_left[0]+int(i*dw-1),tmpp],dtype="double")
                        #print(ptrr_tmp)
                        ptrr = np.append(ptrr,[ptrr_tmp],axis=0)
                        i = i+1  
           
  
            
            # gate towards foward
            if np.abs(angle)<=3:
                
                if w>20 and h>20 and x+w<960 and y+h<720:
                    while i < seg:
                        q = 0
                        cnttt = 0
                        tmpp = 0
                        while q < h:

                            if closing[y+q][x+int(i*dw-1)] != 0:
                                tmpp = tmpp + y+q
                                cnttt =cnttt+1
                            
                            q = q+1
                            
                        if cnttt>0:
                            tmpp = int(tmpp/cnttt)
                        
                        ptrr_tmp = np.array([x+int(i*dw-1),tmpp],dtype="double")
                        #print(ptrr_tmp)
                        ptrr = np.append(ptrr,[ptrr_tmp],axis=0)
                        i = i+1  

            ptrr= np.append(ptrr,end_pt,axis=0)
            ptrr[int(len(ptrr)/2)][0] = tmp_t[0]
            ptrr[int(len(ptrr)/2)][1] = tmp_t[1]
            
            # from ptrr to estimated perspective bound box

            # a given curve
            cv.polylines(res,[np.int32(self.ptr)],True,(0,255,0))
            iipts = np.int32(self.ipts)
            cv.polylines(res,[iipts],True,(0,255,255))
            #print(len(ptrr))

            #print("len of ptrr:"+str(len(ptrr)))
            #print(box_num)
            a_ptr = ptrr

            p_space = np.linspace(-0.5,0.5,100)
            loss = 10e7
            p_ans = 0.0
            if len(a_ptr) >=10:
                for p in p_space:
                    #print("current p ="+str(p))
                    loss_tmp = self.loss_func(p,a_ptr)
                    #print("loss="+str(loss_tmp))
                    if loss_tmp <= loss:
                        p_ans = p
                        loss = loss_tmp


                p_ans = [(p_ans + - angle/180*pi )/2]

                if len(self.Arrp)<5:
                    #print(len(self.Arrtsl))
                    #print(self.Arrtsl)
                    self.Arrp = np.append(self.Arrp,[p_ans],axis=0)
                else:
                    #print("ss")
                    self.Arrp = np.append(self.Arrp,[p_ans],axis=0)
                    self.Arrp = np.delete(self.Arrp,0,0)
                
                p_ans = np.sum(self.Arrp)/len(self.Arrp)
                #print ("p_ans:" +str(p_ans))
                #print ("angle:" +str(angle/180*pi))
                opt_A = [a_ptr[0][0],a_ptr[0][1]]
                opt_D = [a_ptr[int(len(a_ptr)-1)][0], a_ptr[int(len(a_ptr)-1)][1]]

                opt_E = [a_ptr[int(len(a_ptr)/2)][0], a_ptr[int(len(a_ptr)/2)][1]]
                #print(a_ptr[10])

                opt_B = [opt_A[0], opt_E[1] - p_ans * (a_ptr[int(len(a_ptr)/2)][0] - opt_A[0]) ]
                opt_C = [opt_D[0], opt_E[1] - p_ans * (a_ptr[int(len(a_ptr)/2)][0] - opt_D[0]) ]
                #print(p_ans)
                opts = np.float32([opt_A,opt_B,opt_C,opt_D])
                for gg in opts:
                    cv.circle(res, (int(gg[0]), int(gg[1])), 8, (180, 40, 255), 2)
                    #print(a_ptr)

                M_inv = cv.getPerspectiveTransform(opts, self.ipts)

                for tmp_pt in a_ptr:
                    aa = self.warpersp(tmp_pt, M_inv)
                    try:
                        cv.circle(res, (int(aa[0]), int(aa[1])), 3, (0, 0, 255), 2)
                        cv.circle(res,(int(tmp_pt[0]),int(tmp_pt[1])), 3, (0,0,255), -1)
                    except:
                        pass

                oopts = np.int32(opts)
                cv.polylines(res, [oopts], True, (255, 255, 255),3)
                
                img_ptrs = np.array([[0,0]],dtype="double")
                M = cv.getPerspectiveTransform(self.ipts,opts)

                opt_AA = [opt_A[0]-x_err,opt_A[1]-y_err]
                opt_AB = [opt_B[0]-x_err,opt_B[1]-y_err]
                opt_AC = [opt_C[0]-x_err,opt_C[1]-y_err]
                opt_AD = [opt_D[0]-x_err,opt_D[1]-y_err]

                opts = np.float32([opt_AA,opt_AB,opt_AC,opt_AD])
                M2 = cv.getPerspectiveTransform(self.ipts,opts)


                for tmp_pt in self.ptr:
                    aa = self.warpersp(tmp_pt, M)
                    bb = self.warpersp(tmp_pt, M2)
                    try:
                        cv.circle(res, (int(aa[0]), int(aa[1])), 3, (0, 255, 255), 2)
                        cv.circle(res, (int(bb[0]), int(bb[1])), 3, (0, 255, 255), 2)
                    except:
                        pass
                    img_ptrs = np.append(img_ptrs, [ [ aa[0], aa[1] ] ], axis=0)
                    img_ptrs = np.append(img_ptrs, [ [ bb[0], bb[1] ] ], axis=0)
                img_ptrs = np.delete(img_ptrs,0,0)

                #print(ptrr)
                #for gg in ptrr:
                #    #print(gg)
                #    cv.circle(res,(int(gg[0]),int(gg[1])), 3, (0,0,255), -1)
   
                image_points = img_ptrs

            
            
         
            #print(model_points)
            #[-0.735 -0.49  -0.245  0.     0.245  0.49   0.735]
            #xt = np.linspace(-0.75, 0.75, 7,endpoint=True)
            #yt = [-0.65/2,0.3, 0.5, 0.65,0.5,0.3,-0.65/2]
            #print(len(model_points),len(image_points))


            if len(image_points) >= 10:
                (success, rotation_vector, translation_vector) = cv.solvePnP(self.model_points, image_points, camera_matrix, dist_coeffs, flags=cv.SOLVEPNP_DLS)

                if (rotation_vector is not None) and (translation_vector is not None) and (np.sum(np.absolute(translation_vector))<=100) :
                    #cv.drawFrameAxes(res, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 1,4)
                    if flag == False:
                        cv.drawFrameAxes(res, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 1,4)
                        pass
                    a = np.array([translation_vector[0][0],translation_vector[1][0],translation_vector[2][0]],dtype="double")
                    #print(a)

                    if len(self.Arrtsl)<5:
                        #print(len(self.Arrtsl))
                        #print(self.Arrtsl)
                        self.Arrtsl = np.append(self.Arrtsl,[a],axis=0)
                    else:
                        #print("ss")
                        self.Arrtsl = np.append(self.Arrtsl,[a],axis=0)
                        self.Arrtsl = np.delete(self.Arrtsl,0,0)
                    # print(np.sum(self.Arrtsl,axis=0)/20.0)

                    translation_vector = np.sum(self.Arrtsl,axis=0)/50.0

                    pnp_data = PoseStamped()
                    pnp_data.header.frame_id = 'optical'
                   # print("rotation_vector:")
                   # print(rotation_vector)
                    

                    #print("transl_vector:")
                    #print(translation_vector)

                    R,_ = cv.Rodrigues(rotation_vector)
                    #print("rotation_matrix:")
                    #print(R)

                    #translation_vector = np.reshape(3,-1)
                    #translation_vector = - np.dot(np.transpose(R),translation_vector)
                    
                    #print("transl_vector_new:")
                    #print(translation_vector)
                    #translation_vector = np.dot(rotation_vector)    

                    pnp_data.pose.position.x = translation_vector[0]
                    pnp_data.pose.position.y = translation_vector[1]
                    pnp_data.pose.position.z = translation_vector[2]

                    #q = tf_transformations.quaternion_from_euler(rotation_vector[0][0],rotation_vector[1][0],rotation_vector[2][0])
                    q = self.rot2Quat(R)

                    _,goal_yaw,_ = self.euler_from_quaternion(q[0],q[1],q[2],q[3])
                    #tmp = np.abs(goal_yaw -self.state_yaw)
                    q_rot2 = tf_transformations.quaternion_from_euler(0,pi, 0)
                    #if tmp >= pi:
                    #    tmp = 2 * pi -tmp
                    if goal_yaw>pi/2 or goal_yaw<-pi/2:
                        q = tf_transformations.quaternion_multiply(q_rot2, q)
                        print("inverse")
                    #print("goal_yaw = "+str(goal_yaw))
                    #print("state_yaw = "+str(self.state_yaw))
                    
                    
                    pnp_data.pose.orientation.x = q[0]
                    pnp_data.pose.orientation.y = q[1]
                    pnp_data.pose.orientation.z = q[2]
                    pnp_data.pose.orientation.w = q[3]

                    #print('this is pnp')
                    #print(pnp_data)
                    

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    #print(t.header.stamp.sec)
                    
                    t.header.frame_id = 'optical' # for simulation
                    
                    #t.header.frame_id = 'camera_frame'  # for real

                    t.child_frame_id = 'gate'
                
                    # Rotate the previous pose by 180* about X
                
                    t.transform.translation.x = translation_vector[0]
                    t.transform.translation.y = translation_vector[1]
                    t.transform.translation.z = translation_vector[2]

                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
          


                    if flag == False:
                        #cv.drawFrameAxes(res, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 1,4)
                        #print("output")
                        self.br.sendTransform(t)
                        self.pub_pnp.publish(pnp_data)
                else: 
                    #print("shi_bai_le")
                    pass
                    #self.pub_pnp.publish(Pose())

            
            
        #bounding box  #through awawy those little contours
        #

        wid = int(hsv.shape[1] / 2) 
        hgt = int(hsv.shape[0] / 2 )

        sres = cv.resize(res,(wid,hgt))

        try:
            ros_image = self.bridge.cv2_to_imgmsg(sres, encoding="bgr8")
        except CvBridgeError as e:
           print(e)
        self.hsv_pub.publish(ros_image)

        #cv.waitKey(3) 

    def rot2Quat(self,M1):
        r = np.math.sqrt(float(1)+M1[0,0]+M1[1,1]+M1[2,2])*0.5
        i = (M1[2,1]-M1[1,2])/(4*r)
        j = (M1[0,2]-M1[2,0])/(4*r)
        k = (M1[1,0]-M1[0,1])/(4*r)
        return(i,j,k,r)
    

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



    def warpersp(self,ptrr, M):
        src = ptrr
        dst = [0, 0]
        dst[0] = (M[0][0] * src[0] + M[0][1] * src[1] + M[0][2]) / (M[2][0] * src[0] + M[2][1] * src[1] + M[2][2])
        dst[1] = (M[1][0] * src[0] + M[1][1] * src[1] + M[1][2]) / (M[2][0] * src[0] + M[2][1] * src[1] + M[2][2])
        return dst
        pass

    def loss_func(self,p,a_ptr):

        opt_A = [a_ptr[0][0],a_ptr[0][1]]
        opt_D = [a_ptr[int(len(a_ptr)-1)][0], a_ptr[int(len(a_ptr)-1)][1]]

        opt_E = [a_ptr[int(len(a_ptr)/2)][0], a_ptr[int(len(a_ptr)/2)][1]]
        #print(a_ptr[10])

        opt_B = [opt_A[0], opt_E[1] - p * (a_ptr[int(len(a_ptr)/2)][0] - opt_A[0]) ]
        opt_C = [opt_D[0], opt_E[1] - p * (a_ptr[int(len(a_ptr)/2)][0] - opt_D[0]) ]

        opts = np.float32([opt_A,opt_B,opt_C,opt_D])
        #oopts = np.int32(opts)
        #cv.polylines(img, [oopts], True, (0, 255, 25))
        M_inv = cv.getPerspectiveTransform(opts, self.ipts)
        
        sum = 0
        for tmp in a_ptr:
            aa = self.warpersp(tmp, M_inv)
            xt = aa[0]
            yt = -(self.coeff[0]*xt*xt*xt*xt + self.coeff[1]*xt*xt*xt + self.coeff[2]*xt*xt + self.coeff[3]*xt + self.coeff[4])
            sum = sum + (yt - aa[1])*(yt - aa[1])

            #cv.circle(img, (int(aa[0]), int(aa[1])), 3, (0, 0, 255), 2)
        return sum





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
