#!/usr/bin/env python

import rospy
import numpy
import threading

from std_msgs.msg import Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class SteeringCalibrator(object):
   def __init__(self):
      self.timeout = rospy.get_param("~timeout", 2.0)
      self.timeout = rospy.Duration(self.timeout)

      self.speed = rospy.get_param("~speed", 0.5)

      self.pub = rospy.Publisher("cmd_vel", Twist)
      self.offset = None
      self.offset_pub = rospy.Publisher("steering_offset", Int8)

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
      self.offset_pub.publish(msg)

   def pub_speed(self, speed=None):
      if speed is None:
         speed = self.speed
      msg = Twist()
      msg.linear.x = speed 
      self.pub.publish(msg)

   def run(self):
      # set up offset list to try
      offsets = list(range(-10, 10))
      offsets.extend(list(range(10, -10, -1)))
      offset_index = 0
      rospy.loginfo("Calibrating steering servo for %s seconds" % (
               str(self.timeout*len(offsets)) ))
      # publish zero offset to start
      self.offset = offsets[offset_index]
      self.pub_offset()

      # reset internal state while we get moving

      # publish velocity command
      self.pub_speed()
      # sleep a little to let things get moving
      rospy.sleep(1.0)
      self.pub_offset()

      start = rospy.Time.now()
      now = rospy.Time.now()
      while offset_index < len(offsets) and not rospy.is_shutdown():
         # publish an adjustment to the steering angle
         self.offset = offsets[offset_index]
         self.pub_offset()

         # status update: completion percentage and current adjustment
         pct = 100.0 * float(offset_index) / len(offsets)
         rospy.loginfo("%5.2f%%" % ( pct ))

         # publish velocity command
         self.pub_speed()
         # sleep and repeat
         rospy.sleep(self.timeout)
         offset_index += 1

      # publish a stop message
      self.pub_speed(0.0)
      rospy.loginfo("100.00%%")


def main():
   rospy.init_node("steering_cal")
   cal = SteeringCalibrator()
   cal.run()

if __name__ == '__main__':
   main()
