#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

class Path_Planning {
public:
  Path_Planning();

  ros::NodeHandle nh;

  void poserefCb(const geometry_msgs::Pose &pose);
  void odomCb(const nav_msgs::Odometry &odo);
  void load_pose(double X, double Y, double Z);
  void position_control(void);

private:
  ros::Subscriber odom_sub, poseref_sub, poseout_sub;
  ros::Publisher poseref_pub, vel_pub;

  geometry_msgs::Twist velIn;
  geometry_msgs::Pose pose_ref, pose_out;
  nav_msgs::Odometry odo_msg;
  double K;
};

#endif // PATH_PLANNING_HPP
