#ifndef TEST_CONTROLLER_HPP
#define TEST_CONTROLLER_HPP

#include <ardrone_autonomy/Navdata.h>
#include <ardrone_velocity/path_planning.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Test types
#define WITHOUT_CONTROL 0
#define VEL_CONTROL 1
#define POSE_CONTROL 2

// Path types
#define STRAIGHTLINE 0
#define SQUARE 1
#define STATIC 2
#define LEFTLINE 3

class TestController {
public:
  TestController();
  ros::NodeHandle nh;

  ros::Subscriber raw_odom_sub;
  ros::Publisher vel_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher reset_pub;
  ros::Publisher poseref_pub;
  ros::Publisher raw_error_pub;
  ros::Publisher raw_percent_error_pub;

  void raw_odomCb(const nav_msgs::Odometry &odo_msg);
  void land(void);
  void takeoff(void);
  void hover(void);
  void load_vel(double linX, double linY, double linZ, double angZ);
  void load_pose(double Xpos, double Ypos, double Zpos);
  void test();
  void finish(void);
  void init(void);

private:
  int test_type, path_type;
  geometry_msgs::Pose targetpose;
  nav_msgs::Odometry m_raw_odo_msg, m_raw_error_msg, m_raw_percent_error_msg;
  double speed, hovertime, sleeptime, error_x, error_y, error_z;
};
#endif // TEST_CONTROLLER_HPP
