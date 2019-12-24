#!/usr/bin/env python

from __future__ import print_function
import math
import numpy as np
import rospy
from visualization_msgs.msg import MarkerArray

import time

# Brings in the SimpleActionClient
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.0
    goal.target_pose.pose.orientation.w = 0.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()



marker_array = 0

def marker_callback (ros_markers):
    # print ('got an image')
    global marker_array
    marker_array = ros_markers.markers
    


rospy.init_node('actions_manager', anonymous = True)

# # subscibe to map
img_sub = rospy.Subscriber('/explore/frontiers', MarkerArray, marker_callback)
ros_map = rospy.wait_for_message('/explore/frontiers', MarkerArray)


np.set_printoptions(threshold=np.inf)

while not rospy.is_shutdown():

    print(len(marker_array))
    if len(marker_array) <= 2:
        print(movebase_client())
        break;

