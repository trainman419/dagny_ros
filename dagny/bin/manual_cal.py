#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist,Vector3,Vector3Stamped

x = []
y = []
theta = 0
theta_start = None

enc = 0
enc_start = 0

def mag_cb(msg):
    global x
    global y
    x.append(msg.vector.x)
    y.append(msg.vector.y)

def enc_cb(msg):
    if enc_start != 0:
       enc = msg.count
    else:
       enc_start = msg.count


if __name__ == '__main__':
    rospy.init_node("compass_cal")

    pub = rospy.Publisher("compass_cal", Vector3)
    rospy.sleep(5)
    x_cal = float(sys.argv[1])
    print "X: %f"%(x_cal)
    
    y_cal = float(sys.argv[2])
    print "Y: %f"%(y_cal)
    
    cal = Vector3()
    cal.x = x_cal
    cal.y = y_cal

    print cal

    pub.publish(cal)
    
    rospy.sleep(1)

