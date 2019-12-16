#!/usr/bin/env python

import math
import random
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

#matplotlib inline
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


cv_img = 0

def img_callback (ros_img):
    # print ('got an image')
    global bridge, cv_img
    try: 
        cv_img = bridge.imgmsg_to_cv2(ros_img,"rgb8")
    except CvBridgeError as e:
        print (e)

bridge = CvBridge()

rospy.init_node('img', anonymous = True)
img_sub = rospy.Subscriber('camera/image_raw/', Image, img_callback)
ros_img = rospy.wait_for_message('camera/image_raw/', Image)

while not rospy.is_shutdown():
    img = cv_img
    img = cv2.resize(img, (300, 300),interpolation = cv2.INTER_AREA) # for later fire detection
    height = img.shape[0]
    width = img.shape[1]
    cv2.imshow('Scanning for fire', img)
    

    
    if cv2.waitKey(1) == 27: 
        cv2.destroyAllWindows()
        break  # esc to quit
