#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
import numpy as np
from matplotlib import pyplot as plt

class MapClientLaserScanSubscriber(object):

    def __init__(self):
        rospy.Subscriber('/scan',LaserScan,self.get_scan)
        # print 'Debug scan'
        self.z = rospy.wait_for_message("/scan",LaserScan)
        #print self.z

        
    def get_scan(self, msg):  # callback function for LaserScan topic
        self.z = msg

    

    