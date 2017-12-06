#ifndef POSE_CONTROL_HPP
#define POSE_CONTROL_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdio.h>

#include <ardrone_velocity_ekf/test_controller.hpp>

class Pose_Control {
public:
  Pose_Control();

  ros::NodeHandle nh;
  ros::Subscriber odom_sub, poseref_sub;
  ros::Publisher veltoPID_pub;

  void poserefCb(const geometry_msgs::Pose &pose_in);
  void odomCb(const nav_msgs::Odometry &odo);
  void load_pose(double X, double Y, double Z);
  void position_control(void);
  int test_type;

private:
  geometry_msgs::Twist velInPID;
  geometry_msgs::Pose current_pose_ref, pose_out;
  nav_msgs::Odometry odo_msg;
  double K, tol, tolz;
  double distX, distY, distZ, error_dist;
};

#endif // POSE_CONTROL_HPP
