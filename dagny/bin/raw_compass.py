#!/usr/bin/env python

from math import atan2, pi

import rospy
import tf

# messages
from geometry_msgs.msg import Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu

# observed min and max values
x_min =  10.0
x_max = -10.0
y_min =  10.0
y_max = -10.0

x_center = 0.1
x_scale = 1.0

y_center = -0.1
y_scale = 1.0

# compass std dev in degrees
compass_dev = 0.1

# compass variance in radians
compass_var = compass_dev *pi / 180.0
compass_var = compass_var * compass_var

autocompute = False

imu_pub = None

def magnetic_cb(msg):
    global x_max, x_min, y_max, y_min, x_center, y_center, x_scale, y_scale
    if autocompute:
        x_max = max(x_max, msg.vector.x)
        x_min = min(x_min, msg.vector.x)

        y_max = max(y_max, msg.vector.y)
        y_min = min(y_min, msg.vector.y)

        x_center = (x_max + x_min) / 2.0
        y_center = (y_max + y_min) / 2.0

        x_scale = x_max - x_min
        y_scale = y_max - y_min

    x = (msg.vector.x - x_center) / x_scale
    y = (msg.vector.y - y_center) / y_scale

    heading = atan2(x, y)

    imu = Imu()
    imu.header = msg.header
    imu.header.frame_id = 'odom'
    q = tf.transformations.quaternion_from_euler(0, 0, heading)
    imu.orientation = Quaternion(*q)

    imu.orientation_covariance = [ compass_var, 0.0, 0.0,
                                   0.0, compass_var, 0.0,
                                   0.0, 0.0, compass_var ]

    imu.angular_velocity_covariance = [ -1, 0, 0,
                                         0, 0, 0,
                                         0, 0, 0 ]

    imu.linear_acceleration_covariance = [ -1, 0, 0,
                                            0, 0, 0,
                                            0, 0, 0 ]

    imu_pub.publish(imu)


def main():
    global imu_pub
    rospy.init_node("raw_compass")

    imu_pub = rospy.Publisher("compass_imu", Imu)

    mag_sub = rospy.Subscriber("magnetic", Vector3Stamped, magnetic_cb)

    rospy.spin()

if __name__ == '__main__':
    main()
