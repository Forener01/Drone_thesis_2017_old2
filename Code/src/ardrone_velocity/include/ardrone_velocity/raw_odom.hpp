#ifndef RAW_ODOM_HPP
#define RAW_ODOM_HPP

#include <ardrone_velocity/test_controller.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class RawOdom {
public:
  RawOdom();
  ros::NodeHandle nh;

  ros::Subscriber raw_odom_sub;
  ros::Publisher raw_error_pub, raw_percent_error_pub;

  void raw_odomCb(const nav_msgs::Odometry &odo_msg);

private:
  double error_x, error_y, error_z;
  nav_msgs::Odometry m_raw_odo_msg;
  geometry_msgs::Twist m_raw_error_msg, m_raw_percent_error_msg;
};

#endif // RAW_ODOM_HPP