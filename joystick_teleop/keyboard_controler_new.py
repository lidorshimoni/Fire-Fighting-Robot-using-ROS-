#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import tty
import sys
import termios


def main ():
  
  try: 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.init_node('keyboard', anonymous=True)
    
    rate = rospy.Rate(90)

    x = 0
    num = 0

    while not rospy.is_shutdown() and x != chr(27): # ESC to close

      vel  = Twist()
      num+=1

      x = getch()
      print(x)

      if x == 'w':
        vel.linear.x = 2.0 
      elif x == ('s'):
        vel.linear.x = -2.0
      elif x == ('a'):
        vel.angular.z = 2.0
      elif x == ('d'):
        vel.angular.z = -2.0
      elif x == ' ':
        vel.linear.x = 0
        vel.angular.z = 0

      print("forward: ", vel.linear.x)
      print("rotate: ", vel.angular.z)

      pub.publish(vel)
      
      #rate.sleep()

    rospy.spin()
    
  except Exception as e:
    print(e)
  
  finally:
    vel.linear.x = 0; vel.angular.z = 0
    vel.linear.y = 0; vel.linear.z = 0; vel.angular.x = 0; vel.angular.y = 0
    pub.publish(vel)


def getch():
  fd = sys.stdin.fileno()
  old = termios.tcgetattr(fd)
  try:
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
  return ch



if __name__=="__main__":
  
  #try:
    main()
  #except rospy.ROSInterruptException:
   #pass
