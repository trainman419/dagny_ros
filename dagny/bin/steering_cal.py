#!/usr/bin/env python

import rospy
import numpy
import threading

from std_msgs.msg import Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class SteeringCalibrator(object):
   def __init__(self):
      self.timeout = rospy.get_param("~timeout", 10.0)
      self.timeout = rospy.Duration(self.timeout)

      self.rate = rospy.Rate(4.0)

      self.imu_lock = threading.Lock()
      self.imu_buffer = []

      self.sub = rospy.Subscriber("imu", Imu, self.imu_cb)
      self.pub = rospy.Publisher("cmd_vel", Twist)
      self.offset_pub = rospy.Publisher("steering_offset", Int8)

   def imu_cb(self, msg):
      self.imu_lock.acquire()
      self.imu_buffer.append(msg)
      self.imu_lock.release()

   def pub_offset(self, offset):
      msg = Int8()
      msg.data = offset
      self.offset_pub.publish(msg)

   def pub_speed(self, speed):
      msg = Twist()
      msg.linear.x = speed 
      self.pub.publish(msg)

   def run(self):
      rospy.loginfo("Calibrating steering servo for %s seconds" % (
               str(self.timeout) ))
      # TODO: publish zero offset to start
      offset = 0
      self.pub_offset(offset)

      # publish velocity command
      self.pub_speed(1.5)
      # sleep a little to let things get moving
      rospy.sleep(0.1)

      offset_drift = {}

      start = rospy.Time.now()
      now = rospy.Time.now()
      while now < start + self.timeout and not rospy.is_shutdown():
         # lock the imu buffer, avarage the yaw rate, and clear the buffer
         self.imu_lock.acquire()
         yaw_samples = len(self.imu_buffer)
         if yaw_samples > 0:
            average_yaw = numpy.mean([msg.angular_velocity.z for msg in 
                  self.imu_buffer])
            self.imu_buffer = []
         self.imu_lock.release()

         if yaw_samples > 0:
            rospy.loginfo("Average yaw %f over %d samples" % (
                     average_yaw, yaw_samples ))

            # put the average yaw rate into a separate buffer
            if offset not in offset_drift:
               offset_drift[offset] = numpy.array([])

            offset_drift[offset] = numpy.append(offset_drift[offset],
                                                average_yaw)

            # look at the last few average samples to determine how to adjust
            # the steering angle
            # positive yaw corresponds to a positive offset
            median_offset = dict([ (o, numpy.median(offset_drift[o])) for o in
                                   offset_drift ])
            print "Median offsets:"
            print median_offset

            if len(median_offset) < 2:
               # bootstrap with a random guess to get the slope
               if median_offset[0] < 0:
                  offset = 2
               else:
                  offset = -2
            else:
               # do we have an inversion point?
               # find minimum positive value
               min_pos = None
               pos = 1000
               max_neg = None
               neg = -1000
               for o in median_offset:
                  m = median_offset[o]
                  if m > 0 and m < pos:
                     pos = m
                     min_pos = o
                  if m < 0 and m > max_neg:
                     neg = m
                     max_neg = o

               if min_pos is not None and max_neg is not None:
                  # yes; zero in on inversion point
                  rospy.loginfo("Inversion found; improving approximation")
               else:
                  # no; approximate slope and try an entirely new offset
                  rospy.loginfo("Looking for inversion point")

            # publish an adjustment to the steering angle
            rospy.loginfo("Trying offset %d" % ( offset ))
            self.pub_offset(offset)
         else:
            rospy.logwarn("Didn't get any IMU samples!")

         # status update: completion percentage and current adjustment
         elapsed = now - start
         pct = 100.0 * elapsed.to_sec() / self.timeout.to_sec()
         rospy.loginfo("%5.2f%%" % ( pct ))

         # publish velocity command
         self.pub_speed(1.5)
         # sleep and repeat
         self.rate.sleep()
         now = rospy.Time.now()

      # publish a stop message
      self.pub_speed(0.0)
      rospy.loginfo("100.00%%")


def main():
   rospy.init_node("steering_cal")
   cal = SteeringCalibrator()
   cal.run()

if __name__ == '__main__':
   main()
