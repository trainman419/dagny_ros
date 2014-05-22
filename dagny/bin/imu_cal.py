#!/usr/bin/env python

from __future__ import print_function

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
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

   print("Collecting %d samples" % (sample_count) )
   print("Listening for IMU data")

   r = rospy.Rate(1)
   while len(gyro_samples) < sample_count and not rospy.is_shutdown():
      print("%f%%" % ( len(gyro_samples)*100.0/sample_count ) )
      r.sleep()

   # unsubscribe, so we don't get any more callbacks while we're anlyzing data
   imu_sub.unregister()

   if len(gyro_samples) < 1:
      print("ERROR: no samples to analyze. Exiting")
   else:
      print("Analyzing %d samples" % ( len(gyro_samples) ) )

      gyro_samples = numpy.array(gyro_samples).transpose()
      gyro_offset = numpy.mean(gyro_samples, 1)
      print("Gyro bias", gyro_offset)
      print("Gyro covariance", numpy.cov(gyro_samples))

      accel_samples = numpy.array(accel_samples).transpose()
      accel_offset = numpy.mean(accel_samples, 1)
      print("Accelerometer bias", accel_offset)
      print("Accelerometer covaraince", numpy.cov(accel_samples))

      # account for acceleration due to gravity
      accel_offset[2] += 9.81 # 9.81 m/s^2

      imu_cal_pub = rospy.Publisher("imu_cal", Twist)
      rospy.sleep(1)
      if imu_cal_pub.get_num_connections() < 1:
         print("ERROR: could not upload calibration data")
      else:
         cal = Twist()
         cal.linear.x = accel_offset[0]
         cal.linear.y = accel_offset[1]
         cal.linear.z = accel_offset[2]
         cal.angular.x = gyro_offset[0]
         cal.angular.y = gyro_offset[1]
         cal.angular.z = gyro_offset[2]
         print("Sending calibration data", cal)
         imu_cal_pub.publish(cal)


if __name__ == '__main__':
   main()
