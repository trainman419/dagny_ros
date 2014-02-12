/*
 * drive.cpp
 *
 * Drive the robot in a straight line
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char ** argv) {
   ros::init(argc, argv, "steer");

   int steer = 0;

   if( argc >= 2 ) {
      sscanf(argv[1], "%d", &steer);
   }

   ros::NodeHandle n;

   ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("control", 10);

   ros::Rate loop_rate(100);
   ROS_INFO("Setting steering to %d", steer);

   while( ros::ok() ) {
      geometry_msgs::Twist c;
      c.linear.x = 0;
      c.angular.z = steer;
      control_pub.publish(c);

      ros::spinOnce();

      loop_rate.sleep();
   }


   ros::spin();

   return 0;
}
