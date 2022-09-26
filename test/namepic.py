
f=open('/home/hongli/tello_ws/src/tello_mod/test/name.txt','w')

import cv2
import os

def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            f.write(filename+'\n')
    return images
folder = '/home/hongli/catkin_ws/src/test/pic'
load_images_from_folder(folder)