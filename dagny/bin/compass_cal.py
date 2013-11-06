#!/usr/bin/env python

import rospy
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
    sub = rospy.Subscriber("magnetic", Vector3Stamped, mag_cb)
    #sub2 = rospy.Subscriber("encoder", 
    cmd_pub = rospy.Publisher("cmd_vel", Twist)

    cmd = Twist()
    cmd.linear.x = 0.4
    cmd.angular.z = 0.5

    cal = Vector3()
    pub.publish(cal)
   
    rospy.sleep(1)

    for i in range(15):
        cmd_pub.publish(cmd)
        rospy.sleep(1)

    x_min = min(x)
    x_max = max(x)
    x_cal = (x_min + x_max) / 2.0
    print "X: %f, %f: %f"%(x_min, x_max, x_cal)
    
    y_min = min(y)
    y_max = max(y)
    y_cal = (y_min + y_max) / 2.0
    print "Y: %f %f: %f"%(y_min, y_max, y_cal)
    
    cal.x = x_cal
    cal.y = y_cal

    print cal

    cmd.linear.x = 0
    cmd.angular.z = 0
    cmd_pub.publish(cmd)
    pub.publish(cal)
    
    rospy.sleep(3)

