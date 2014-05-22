#!/usr/bin/env python

from __future__ import print_function

import rospy
from sensor_msgs.msg import Imu
import numpy

gyro_samples = []
accel_samples = []

def imu_cb(msg):
   gyro = msg.angular_velocity
   gyro_samples.append([gyro.x, gyro.y, gyro.z])

   accel = msg.linear_acceleration
   accel_samples.append([accel.x, accel.y, accel.z])
   pass

def main():
   global gyro_samples
   global accel_samples

   rospy.init_node("imu_cal")

   imu_sub = rospy.Subscriber("imu", Imu, imu_cb)

   # number of samples to collect
   sample_count = rospy.get_param("~samples", 1000)

   # increment
   increment = 50
   i = increment

   print("Collecting %d samples" % (sample_count) )
   print("Listening for IMU data")

   r = rospy.Rate(1)
   while len(gyro_samples) < sample_count and not rospy.is_shutdown():
      if len(gyro_samples) > i :
         print("%f%%" % ( len(gyro_samples)*100.0/sample_count ) )
         i += increment
      r.sleep()

   # unsubscribe, so we don't get any more callbacks while we're anlyzing data
   imu_sub.unregister()

   if len(gyro_samples) < 1:
      print("ERROR: no samples to analyze. Exiting")
   else:
      print("Analyzing %d samples" % ( len(gyro_samples) ) )

      gyro_samples = numpy.array(gyro_samples).transpose()
      print("Gyro bias", numpy.mean(gyro_samples, 1))
      print("Gyro covariance", numpy.cov(gyro_samples))

      accel_samples = numpy.array(accel_samples).transpose()
      print("Accelerometer bias", numpy.mean(accel_samples, 1))
      print("Accelerometer covaraince", numpy.cov(accel_samples))


if __name__ == '__main__':
   main()
