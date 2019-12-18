#!/usr/bin/env python

import math
import random, cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

#matplotlib inline
from matplotlib import pyplot as plt
from PIL import Image
import time

# TODO get pose and combine with map 

map = 0
robot_x = 0
robot_y = 0
robot_theta = 0

def crop_image(img,tol=0):
    # img is 2D image data
    # tol  is tolerance
    mask = img>tol
    return img[np.ix_(mask.any(1),mask.any(0))]

def map_img_callback (ros_img):
    # print ('got an image')
    global map
    map = np.asarray(ros_img.data)
    map = np.reshape(map, (1024, 1024))
    map = map[::-1]
    map = np.rot90(map)
    
    
def pose_callback (ros_img):
    # print ('got pose')
    global robot_theta
    global robot_x
    global robot_y
    pose = ros_img
    robot_x = -int(round(pose.pose.position.y * 10, 1))
    robot_y = -int(round(pose.pose.position.x * 10, 1))
    robot_theta = abs((pose.pose.orientation.z))    


rospy.init_node('map_img', anonymous = True)

# # subscibe to map
img_sub = rospy.Subscriber('/map', OccupancyGrid, map_img_callback)
ros_map = rospy.wait_for_message('/map', OccupancyGrid)

# subscibe to slam pose
pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)
ros_pose = rospy.wait_for_message('/slam_out_pose', PoseStamped)

np.set_printoptions(threshold=np.inf)

while not rospy.is_shutdown():
    # img = crop_image(map, -1).astype(np.uint8)
    img = map.astype(np.uint8)
    # img = cv2.resize(img, dsize=(1024, 1024), interpolation=cv2.INTER_NEAREST)

    img[img==-1] = 255
    img[img==0] = 180
    img[img<180] = 0

    img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
    
    # print("*"*20+ "\n", (np.tan(robot_theta).as_integer_ratio()[1])*5)
    # print((np.tan(robot_theta).as_integer_ratio()[0])*5)
    # print((np.tan(robot_theta).as_integer_ratio()[1])*5)
    # print((np.tan(robot_theta).as_integer_ratio()[0])*5)
    # image = cv2.arrowedLine(img, (512+robot_x, 512+robot_y), (512+robot_x+int(np.tan(robot_theta).as_integer_ratio()[1])*10, 512+robot_y+int(np.tan(robot_theta).as_integer_ratio()[0])*10), [255,0,0], 2)

    # img = cv2.arrowedLine(img, (512+robot_x, 512+robot_y), (int(512+robot_x+ (10*np.cos(robot_theta))), int(512+robot_y+ (10*np.sin(robot_theta)))), [0,0,255], 2)
    img = cv2.circle(img, (512+robot_x,512+robot_y), 2, [250,0,0], 1)
    cv2.imshow('Getting map', img)

    
    if cv2.waitKey(1) == 27: 
        cv2.destroyAllWindows()
        break  # esc to quit
