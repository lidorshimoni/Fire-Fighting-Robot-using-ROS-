#!/usr/bin/env python

import math
import random
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

#matplotlib inline
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


cv_img = 0
robot_vel = 0

def img_callback (ros_img):
    # print ('got an image')
    global bridge, cv_img
    try: 
        cv_img = bridge.imgmsg_to_cv2(ros_img,"rgb8")
    except CvBridgeError as e:
        print (e)
    
def robot_vel_callback(twist):
    global robot_vel
    robot_vel = twist

bridge = CvBridge()

rospy.init_node('detect', anonymous = True)
# img_sub = rospy.Subscriber('camera/image_raw/', Image, img_callback)
img_sub = rospy.Subscriber('detection/object/detection_visualization/', Image, img_callback)


# ros_img = rospy.wait_for_message('camera/image_raw/', Image)
ros_img = rospy.wait_for_message('detection/object/detection_visualization/', Image)

img_sub = rospy.Subscriber('/cmd_vel', Twist, robot_vel_callback)
ros_img = rospy.wait_for_message('/cmd_vel', Twist)

while not rospy.is_shutdown():
    img = cv_img
    # img = cv2.resize(img, (300, 300),interpolation = cv2.INTER_AREA) # for later fire detection
    height = img.shape[0]
    width = img.shape[1]
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    fontScale              = 0.5
    fontColor              = (0,0,255)
    lineType               = 2

    cv2.putText(img,'linear: ' + str(round(robot_vel.linear.x, 2)*3.6), 
    (10,20), 
    font, 
    fontScale,
    fontColor,
    lineType)

    cv2.putText(img,"angular: " + str(round(robot_vel.angular.z, 2)*3.6), 
    (10,35), 
    font, 
    fontScale,
    fontColor,
    lineType)
    cv2.imshow('Scanning for fire', img)

    
    if cv2.waitKey(1) == 27: 
        cv2.destroyAllWindows()
        break  # esc to quit
