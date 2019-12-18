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
    
    
def quaternion_to_euler(q):

    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def pose_callback (ros_img):
    # print ('got pose')
    global robot_theta
    global robot_x
    global robot_y
    pose = ros_img
    robot_x = -int(round(pose.pose.position.y * 10, 1))
    robot_y = -int(round(pose.pose.position.x * 10, 1))
    robot_theta = -(quaternion_to_euler(pose.pose.orientation)[0]+np.pi/2)  


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


    # make RGB - beutify
    img[img==-1] = 255
    img[img==0] = 180
    img[img<180] = 0
    img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)


    # draw robot pos
    circle_r = 2
    arrow_l = 10
    img = cv2.arrowedLine(img, (512+robot_x, 512+robot_y), (int(512+robot_x+ (arrow_l*np.cos(robot_theta))), int(512+robot_y+ (arrow_l*np.sin(robot_theta)))), [0,0,255], 1)
    img = cv2.circle(img, (512+robot_x,512+robot_y), circle_r, [250,0,0], 1)

    # show map
    cv2.imshow('Getting map', img)

    
    if cv2.waitKey(1) == 27: 
        cv2.destroyAllWindows()
        break  # esc to quit
