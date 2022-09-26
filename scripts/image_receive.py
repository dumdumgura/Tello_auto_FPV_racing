
from tkinter import CENTER
from matplotlib.pyplot import text
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

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('image_receive')
        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.listener_callback,
            10)
            
        self.pub = self.create_publisher(BoundBox,'/drone1/hsv_boundbox',10)
        self.hsv_pub = self.create_publisher(Image,'/drone1/hsv',10)
        #self.darknet_ros_sub = self.create_subscription(BoundingBoxes,'/darknet_ros/bounding_boxes',self.darknet_ros_callback,10)
        self.bridge = CvBridge()
        self.pub_pnp = self.create_publisher(PoseStamped, '/pnpPose', 1)
        self.subscription  # prevent unused variable warning

    def darknet_ros_callback(self,msg):

        print("success")

    def listener_callback(self, msg):
        #self.get_logger().info('image received')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        except CvBridgeError as e:
           print(e)
        
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
        
        blur = cv.blur(cv_image,(10,10))
        #blur = cv.GaussianBlur(cv_image,(5,5),0)
        #blur = cv.bilateralFilter(cv_image,9,75,75)
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

        #mask =cv.inRange(hsv,(117,236,75),(179,255,255)) #for tello
       
        #mask =cv.inRange(hsv,(0,57,210),(7,210,255))  #for usb camera
        # define range of blue color in HSV
        
        lower_hue = np.array([117,200,75])
        upper_hue = np.array([180,255,255])
    

        # Threshold the HSV image to get only blue colors
        mask1 = cv.inRange(hsv, lower_hue, upper_hue)

        lower_blue = np.array([0,236,75])
        upper_blue = np.array([30,255,255])
        
        mask2 = cv.inRange(hsv, lower_blue, upper_blue)

    
        mask = mask1+mask2


        kernel = np.ones((7,7),np.uint8)

        
        opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        dilation = cv.dilate(opening,kernel,iterations = 1)
        closing = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # Bitwise-AND mask and original image
        
        res = cv.bitwise_and(cv_image,cv_image, mask = closing)
        
        #imgray = cv.cvtColor(mask, cv.COLOR_BGR2GRAY)
        #ret, thresh = cv.threshold(imgray, 127, 255, 0)
        
        contours, hierarchy = cv.findContours(closing, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
       
        #
        
        
        for cntt in contours:
            cv.drawContours(res, cntt, -1, (0,255,255), 3)   #yellow for all contours
            #area = cv.contourArea(cnt)
            op = 0


        box_num = 0
        area = 0
        ptr_left = np.array([0,0],dtype="double")
        ptr_right = np.array([0,0],dtype="double")


        model_points = np.array([    #x,y,z
                                [0, -0.65/2, -0.7],     # Front Left Corner
                                [0, -0.65/2, 0.7],      # Front Right Corner 
                                [0,  0.65/2,  0],    # Front Top
                                [0, -0.65/2,      0],      # Front bottom
                            ], dtype="double")

        camera_matrix = np.array(
                            [[924.873180, 0.000000, 486.997346 ],[0.000000, 923.504522, 364.308527],[ 0.000000, 0.000000, 1.000000]])
   
        dist_coeffs =np.array([-0.034749, 0.071514, 0.000363, 0.003131, 0.000000])
    

        if len(contours)!=0 :
            cnt = max(contours, key = cv.contourArea)
            area = cv.contourArea(cnt)

        if area > 3000:
            box_num=box_num+1
            cv.drawContours(res, cnt, -1, (0,255,0), 3) #green lines for enough big contours
        
            epsilon = 0.005*cv.arcLength(cnt,True)
            approx = cv.approxPolyDP(cnt,epsilon,True)
            
            tmp = approx[0]
            tmp_l = tmp[0]
            tmp_r = tmp[0]
            tmp_t = tmp[0]
            tmp_b = tmp[0]
            #print(tmp)
            #print("tmp="+str(tmp))
            for ptr in approx:
                if ptr[0][0] < tmp_l[0]:
                    tmp_l = ptr[0]
                if ptr[0][0] > tmp_r[0]:
                    tmp_r = ptr[0]
                if ptr[0][1] < tmp_t[1]:
                    tmp_t = ptr[0]
                if ptr[0][1] > tmp_b[1]:
                    tmp_b = ptr[0]
            cv.circle(res,(tmp_l[0],tmp_l[1]), 15, (0,0,255), -1)
            cv.circle(res,(tmp_r[0],tmp_r[1]), 15, (0,0,255), -1)
            cv.circle(res,(tmp_t[0],tmp_t[1]), 15, (0,0,255), -1)
            cv.circle(res,(tmp_b[0],tmp_b[1]), 15, (0,0,255), -1)

            l2b = (tmp_l[0]-tmp_b[0])*(tmp_l[0]-tmp_b[0])+(tmp_l[1]-tmp_b[1])*(tmp_l[1]-tmp_b[1])
            r2b = (tmp_r[0]-tmp_b[0])*(tmp_r[0]-tmp_b[0])+(tmp_r[1]-tmp_b[1])*(tmp_r[1]-tmp_b[1])

            #print("l2b="+str(l2b))
            #print("r2b="+str(r2b))

            if (l2b<=r2b): # gate is toward right
                #cv.line(res,(tmp_b[0],tmp_b[1]),(tmp_r[0],tmp_r[1]),(25,123,0),10)
                #cv.line(res,(tmp_b[0],tmp_b[1]),(tmp_t[0],tmp_t[1]),(25,123,0),10)
                #cv.line(res,(tmp_r[0],tmp_r[1]),(tmp_t[0],tmp_t[1]),(25,123,0),10)
                #print("l2b>r2b")
                ptr_left = tmp_b
                ptr_right = tmp_r
                image_points = np.array([
                    #[extLeft[0],extLeft[1]],     # Left Extreme of Gate
                    [tmp_l[0],tmp_l[1]],     # Bottom Extreme of Gate (Left Corner)
                    [tmp_b[0],tmp_b[1]],    # Right Extreme of Gate
                    [tmp_t[0],tmp_t[1]],       # Top Extreme of  Gate
                    [1/2*(tmp_b[0]+tmp_l[0]),1/2*(tmp_b[1]+tmp_l[1])],
                    ], dtype="double")
        

            else:
                #cv.line(res,(tmp_b[0],tmp_b[1]),(tmp_l[0],tmp_l[1]),(25,123,0),10)
                #cv.line(res,(tmp_b[0],tmp_b[1]),(tmp_t[0],tmp_t[1]),(25,123,0),10)
                #cv.line(res,(tmp_l[0],tmp_l[1]),(tmp_t[0],tmp_t[1]),(25,123,0),10)
                #print("r2b>l2b")
                ptr_right = tmp_b
                ptr_left = tmp_l

                image_points = np.array([
                    #[extLeft[0],extLeft[1]],     # Left Extreme of Gate
                    [tmp_l[0],tmp_l[1]],     # Bottom Extreme of Gate (Left Corner)
                    [tmp_b[0],tmp_b[1]],    # Right Extreme of Gate
                    [tmp_t[0],tmp_t[1]],       # Top Extreme of  Gate
                    [1/2*(tmp_b[0]+tmp_l[0]),1/2*(tmp_b[1]+tmp_l[1])],
                    ], dtype="double")

            #cv.drawContours(res, tmp_t, -1, (255,255,0), 5)
            #cv.drawContours(res, approx, -1, (255,255,0), 50)     #acid blue for aprox po

            x,y,w,h = cv.boundingRect(approx)   
            cv.rectangle(res,(x,y),(x+w,y+h),(255,0,25),3)        #blue for bounding box
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
            cv.putText(res,text,(x,y+h), font, 1,(255,255,255),4,cv.LINE_AA)
            cv.putText(res,text2,(x,y+h+30), font, 1,(255,255,255),4,cv.LINE_AA)
            
            msg  = BoundBox()
            msg.xmin = x
            msg.ymin = y
            msg.width = w
            msg.height = h
            self.pub.publish(msg)


            #image_points = np.array([
            #        #[extLeft[0],extLeft[1]],     # Left Extreme of Gate
            #        [x,y],     # Bottom Extreme of Gate (Left Corner)
            #        [x+w,y],    # Right Extreme of Gate
            #        [x,y+h],       # Top Extreme of  Gate
            #        [x+w,y+h],
            #        ], dtype="double")
            (success, rotation_vector, translation_vector) = cv.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv.SOLVEPNP_P3P)
           
            if (rotation_vector is not None) or (translation_vector is not None):

                cv.drawFrameAxes(res, camera_matrix, dist_coeffs, rotation_vector, translation_vector, 1,4)
                pnp_data = PoseStamped()
                pnp_data.header.frame_id = 'camera_link'
                print("rotation_vector:")
                print(rotation_vector)
                

                print("transl_vector:")
                print(translation_vector)

                R,_ = cv.Rodrigues(rotation_vector)
                print("rotation_matrix:")
                print(R)

                #translation_vector = np.reshape(3,-1)
                translation_vector = - np.dot(np.transpose(R),translation_vector)
                
                print("transl_vector_new:")
                print(translation_vector)
                #translation_vector = np.dot(rotation_vector)    

                pnp_data.pose.position.x = translation_vector[0][0]
                pnp_data.pose.position.y = translation_vector[1][0]
                pnp_data.pose.position.z = translation_vector[2][0]

                #q = tf_transformations.quaternion_from_euler(rotation_vector[0][0],rotation_vector[1][0],rotation_vector[2][0])
                q = self.rot2Quat(R)

                pnp_data.pose.orientation.x = -q[1]
                pnp_data.pose.orientation.y = -q[2]
                pnp_data.pose.orientation.z = -q[3]
                pnp_data.pose.orientation.w = q[0]

                #print('this is pnp')
                #print(pnp_data)
                self.pub_pnp.publish(pnp_data)
            
            else: 
                print("shi_bai_le")
                #self.pub_pnp.publish(Pose())

            
            
        #bounding box  #through awawy those little contours
        #

        wid = int(hsv.shape[1] / 2) 
        hgt = int(hsv.shape[0] / 2 )
        #print("width="+str(hsv.shape[1])+"height="+str(hsv.shape[0]))
        
        #sframe = cv.resize(cv_image,(wid,hgt))
        #sgaus = cv.resize(blur,(wid,hgt))
        #smask = cv.resize(mask,(wid,hgt))
        sres = cv.resize(res,(wid,hgt))
        #scont = cv.resize(cont,(wid,hgt))

        #cv.imshow('frame',sframe)
        #cv.imshow('mask',smask)
        
        #cv.imshow('gaus',sgaus)
       
        #cv.imshow('mask',mask)
        #cv.imshow('blur',blur)
        #cv.imshow('dil',dilation)
        #cv.imshow('open',opening)
        #k = cv.waitKey(10) & 0xFF
        #cv.imshow('res',sres)
        #cv.destroyAllWindows()
        #cv.imshow("Image window", sframe)
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
