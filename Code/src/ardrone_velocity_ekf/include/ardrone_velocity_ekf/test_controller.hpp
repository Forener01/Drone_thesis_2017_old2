#ifndef TEST_CONTROLLER_HPP
#define TEST_CONTROLLER_HPP

#include <ardrone_autonomy/Navdata.h>
#include <ardrone_velocity_ekf/pose_controller.hpp>
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
#define ROOM_EXIT 4

class TestController {
public:
  TestController();
  ros::NodeHandle nh;

  ros::Publisher vel_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher reset_pub;
  ros::Publisher poseref_pub;

  void land(void);
  void takeoff(void);
  void hover(void);
  void load_vel(double linX, double linY, double linZ, double angZ);
  void load_pose(double Xpos, double Ypos, double Zpos);
  void test();
  void finish(void);
  void init(void);

  int test_type;

private:
  int path_type;
  geometry_msgs::Pose targetpose;

  double speed, hovertime, sleeptime;
};
#endif // TEST_CONTROLLER_HPP
