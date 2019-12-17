#!/usr/bin/env python

import math
import random, cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
# from nav_msgs.msg import slam_out_pose

#matplotlib inline
from matplotlib import pyplot as plt
from PIL import Image
import time

# TODO get /map topic and translate it to array.

map = 0

def map_img_callback (ros_img):
    # print ('got an image')
    global map
    map = np.asarray(ros_img.data)
    map = np.reshape(map, (1024, 1024))
    print("got the map!\n", map.shape)
    
# def pose_callback (ros_img):
#     # print ('got an image')
#     global map
#     map = np.asarray(ros_img.data)
#     map = np.reshape(map, (1024, 1024))
#     print("got the map!\n", map.shape)


rospy.init_node('map_img', anonymous = True)

# subscibe to map
img_sub = rospy.Subscriber('/map', OccupancyGrid, map_img_callback)
ros_map = rospy.wait_for_message('/map', OccupancyGrid)

# # subscibe to slam pose
# img_sub = rospy.Subscriber('/slam_out_pose', slam_out_pose, pose_callback)
# ros_map = rospy.wait_for_message('/slam_out_pose', slam_out_pose)


while not rospy.is_shutdown():

    img = map
    img = np.delete(np.delete(img, [range(400), range(600,1000)], axis=0), [range(400), range(600,1000)], axis=1)
    img[img==-1] = 255
    img[img==0] = 180
    img[img<180] = 0


    # plt.imshow(img, interpolation='nearest')
    # plt.show()    
    cv2.imshow('Getting map', np.uint8(img))


    
    if cv2.waitKey(1) == 27: 
        cv2.destroyAllWindows()
        break  # esc to quit
