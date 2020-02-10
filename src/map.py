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

from fire_fighting_robot.msg import DetectionResult
from fire_fighting_robot.msg import DetectionResults

from sensor_msgs.msg import LaserScan

map = 0
robot_x = 0
robot_y = 0
robot_theta = 0
resolution = 1024
middle = resolution/2
detected_objects=[]
persons = []

def crop_image(img,tol=0):
    # img is 2D image data
    # tol  is tolerance
    mask = img>tol
    return img[np.ix_(mask.any(1),mask.any(0))]

def map_img_callback (ros_img):
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

def pose_callback (pose):
    global robot_theta
    global robot_x
    global robot_y
    robot_x = -int(round(pose.pose.position.y * 10, 1)) + middle
    robot_y = -int(round(pose.pose.position.x * 10, 1)) + middle
    robot_theta = -(quaternion_to_euler(pose.pose.orientation)[0]+np.pi/2)  

class person():
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def get_pos(self) :
        return (self.x, self.y)

    def distance(self, x, y):
        return np.sqrt((self.x-x)**2 + (self.y-y)**2)


class detectedObjectLocation():
    def __init__(self, object_data):
        self.out_class = object_data.out_class
        self.out_score = object_data.out_score
        self.box = object_data.location
        self.angle_in_view = ((self.box[1]/640*120 + self.box[3]/640*120)/2 -60)*3.1416/180 # to rad

def add_person(distance, angle):
    global persons
    x, y = (int(robot_x+ (distance*np.cos(angle))), int(robot_y+ (distance*np.sin(angle))))
    for per in persons:
        if per.distance(x,y) < 10:
            return
    persons.append(person(x, y))




def detection_results_callback (data):
    global detected_objects
    detected_objects = []
    data = data.results
    for obj in data:
        detected_objects.append(detectedObjectLocation(obj))


def laser_scan_callback (scan):
    global detected_objects
    min = scan.angle_min
    inc = scan.angle_increment
    for obj in detected_objects:
        add_person(float(scan.ranges[int((obj.angle_in_view/4.18879)*len(scan.ranges))]) * 2, obj.angle_in_view)


rospy.init_node('map_img', anonymous = True)

# # subscibe to map
img_sub = rospy.Subscriber('/map', OccupancyGrid, map_img_callback)
ros_map = rospy.wait_for_message('/map', OccupancyGrid)

# subscibe to slam pose
pose_sub = rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)
ros_pose = rospy.wait_for_message('/slam_out_pose', PoseStamped)

# subscibe to detection_result
pose_sub = rospy.Subscriber('/detection/object/detection_result', DetectionResults, detection_results_callback)
ros_pose = rospy.wait_for_message('/detection/object/detection_result', DetectionResults)

# subscibe to detection_result
pose_sub = rospy.Subscriber('/scan', LaserScan, laser_scan_callback)
ros_pose = rospy.wait_for_message('/scan', LaserScan)


rate = rospy.Rate(15)
# np.set_printoptions(threshold=np.inf)

arrow_l = 10

# def find_wall(map):
#     pt2 = pt3 = None
#     for distance in range(1,arrow_l*10):
#         if pt2 is None and map[int(robot_x+ (distance*np.cos(robot_theta-1.0472))), int(robot_y+ (distance*np.sin(robot_theta-1.0472))),0] == 0:
#             pt2 = int(robot_x+ (distance*np.cos(robot_theta-1.0472))), int(robot_y+ (distance*np.sin(robot_theta-1.0472)))
#         if pt3 is None and map[int(robot_x+ (distance*np.cos(robot_theta+1.0472))), int(robot_y+ (distance*np.sin(robot_theta+1.0472))), 0] == 0:
#             pt3 = int(robot_x+ (distance*np.cos(robot_theta+1.0472))), int(robot_y+ (distance*np.sin(robot_theta+1.0472)))
#     if not pt2:
#         pt2 = (int(robot_x+ (arrow_l*10*np.cos(robot_theta-1.0472))), int(robot_y+ (arrow_l*10*np.sin(robot_theta-1.0472))))
#     if not pt3:
#         pt3 = (int(robot_x+ (arrow_l*10*np.cos(robot_theta+1.0472))), int(robot_y+ (arrow_l*10*np.sin(robot_theta+1.0472))))
#     return (pt2, pt3)
        



# def draw_field_view(map):

#     overlay = map.copy()
#     pt1 = (robot_x, robot_y)
#     (pt2, pt3) = find_wall(map)
    
#     triangle_cnt = np.array( [pt1, pt2, pt3] )
#     cv2.drawContours(map, [triangle_cnt], 0, (0,255,0, 1), -1)


#     alpha = 0.4  # Transparency factor.

#     # Following line overlays transparent rectangle over the image
#     map = cv2.addWeighted(overlay, alpha, map, 1 - alpha, 0)

#     return map

def draw_persons(img):
    global persons
    for per in persons:
        # img = cv2.circle(img, per.get_pos(), 3, [250,150,0], 1)
        img = cv2.arrowedLine(img, (int(robot_x), int(robot_y)), per.get_pos(), [10,10,60], 1)

    return img


while not rospy.is_shutdown():
    # img = crop_image(map, -1).astype(np.uint8)
    img = map.astype(np.uint8)
    # img = cv2.resize(img, dsize=(1024, 1024), interpolation=cv2.INTER_NEAREST)


    # make RGB - beutify
    img[img==-1] = 255
    img[img==0] = 180
    img[img<180] = 0
    img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
    # img = draw_field_view(img)

    color = [20,150,20]
    # img[img==color] =




    # draw robot pos
    circle_r = 2
    arrow_l = 10
    img = cv2.arrowedLine(img, (int(robot_x), int(robot_y)), (int(robot_x+ (arrow_l*np.cos(robot_theta))), int(robot_y+ (arrow_l*np.sin(robot_theta)))), [255,0,0], 1)
    # print('-'*50)
    # print((robot_x,robot_y), circle_r, [250,0,0], 1)
    # print('-'*50)

    img = cv2.circle(img, (int(robot_x), int(robot_y)), circle_r, [250,0,0], 1)

    img = draw_persons(img)

    # img = cv2.arrowedLine(img, (robot_x, robot_y), (int(robot_x+ (arrow_l*10*np.cos(robot_theta+1.0472))), int(robot_y+ (arrow_l*10*np.sin(robot_theta+1.0472)))), [0,0,255], 1)
    # img = cv2.arrowedLine(img, (robot_x, robot_y), (int(robot_x+ (arrow_l*10*np.cos(robot_theta-1.0472))), int(robot_y+ (arrow_l*10*np.sin(robot_theta-1.0472)))), [0,0,255], 1)
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    fontScale              = 0.5
    fontColor              = (0,0,255)
    lineType               = 2

    cv2.putText(img,"(" + str(round(robot_x-512, 2)) + ", " + str(round(robot_y-512, 2)) + ", " + str(round(robot_theta*57.2958, 2)) + ")", 
    (400+10,400+20), 
    font, 
    fontScale,
    fontColor,
    lineType)

    # show map
    cv2.imshow('Getting map', img)

    
    if cv2.waitKey(1) == 27: 
        cv2.destroyAllWindows()
        break  # esc to quit
    rate.sleep()
