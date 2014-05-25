#!/usr/bin/env python

import rospy
import numpy
import threading

from std_msgs.msg import Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class SteeringCalibrator(object):
   def __init__(self):
      self.timeout = rospy.get_param("~timeout", 8.0)
      self.timeout = rospy.Duration(self.timeout)

      self.speed = rospy.get_param("~speed", 0.5)

      self.rate = rospy.Rate(2.0)

      self.imu_lock = threading.Lock()
      self.imu_buffer = []

      self.sub = rospy.Subscriber("imu", Imu, self.imu_cb)
      self.pub = rospy.Publisher("cmd_vel", Twist)
      self.offset = None
      self.current_offset = None
      self.offset_time = rospy.Time.now()
      self.offset_pub = rospy.Publisher("steering_offset", Int8)

   def imu_cb(self, msg):
      self.imu_lock.acquire()
      diff = (rospy.Time.now() - self.offset_time).to_sec()
      # angular lag while moving is about 0.2-0.3 seconds
      if diff > 0.5:
         self.imu_buffer.append(msg)
      self.imu_lock.release()

   def pub_offset(self):
      # limit offset to +/- 10
      if self.offset > 10:
         rospy.logwarn("Capping offset to 10")
         self.offset = 10
      if self.offset < -10:
         rospy.logwarn("Capping offset to -10")
         self.offset = -10

      rospy.loginfo("Trying offset %d" % ( self.offset ))

      msg = Int8()
      msg.data = self.offset
      if self.current_offset != self.offset:
         self.imu_lock.acquire()
         self.offset_time = rospy.Time.now()
         self.current_offset = self.offset
         self.imu_buffer = []
         self.imu_lock.release()
      self.offset_pub.publish(msg)

   def pub_speed(self, speed=None):
      if speed is None:
         speed = self.speed
      msg = Twist()
      msg.linear.x = speed 
      self.pub.publish(msg)

   def run(self):
      rospy.loginfo("Calibrating steering servo for %s seconds" % (
               str(self.timeout) ))
      # publish zero offset to start
      self.offset = 0
      self.pub_offset()

      # reset internal state while we get moving
      self.current_offset = None

      # publish velocity command
      self.pub_speed()
      # sleep a little to let things get moving
      rospy.sleep(1.0)
      self.pub_offset()

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
            if self.offset not in offset_drift:
               offset_drift[self.offset] = numpy.array([])

            offset_drift[self.offset] = numpy.append(offset_drift[self.offset],
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
               if median_offset[0] > 0:
                  self.offset = 2
               else:
                  self.offset = -2
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
                  if min_pos - max_neg > 1:
                     self.offset = min_pos - 1
                  else:
                     if len(offset_drift[min_pos]) > len(offset_drift[max_neg]):
                        self.offset = max_neg
                     else:
                        self.offset = min_pos
               else:
                  # no; approximate slope and try an entirely new offset
                  rospy.loginfo("Looking for inversion point")
                  self.offset = self.offset * 2
                  
            # publish an adjustment to the steering angle
            self.pub_offset()
         else:
            rospy.logwarn("Didn't get any IMU samples!")

         # status update: completion percentage and current adjustment
         elapsed = now - start
         pct = 100.0 * elapsed.to_sec() / self.timeout.to_sec()
         rospy.loginfo("%5.2f%%" % ( pct ))

         # publish velocity command
         self.pub_speed()
         # sleep and repeat
         self.rate.sleep()
         now = rospy.Time.now()

      # publish a stop message
      self.pub_speed(0.0)
      rospy.loginfo("100.00%%")

      # dump all of the raw data we collected
      print offset_drift


def main():
   rospy.init_node("steering_cal")
   cal = SteeringCalibrator()
   cal.run()

if __name__ == '__main__':
   main()
