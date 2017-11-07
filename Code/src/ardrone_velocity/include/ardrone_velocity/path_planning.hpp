#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdio.h>

class Path_Planning {
public:
  Path_Planning();

  ros::NodeHandle nh;

  void poserefCb(const geometry_msgs::Pose &pose_in);
  void odomCb(const nav_msgs::Odometry &odo);
  void load_pose(double X, double Y, double Z);
  void position_control(void);
  int test_type;

private:
  ros::Subscriber odom_sub, poseref_sub, poseout_sub;
  ros::Publisher poseref_pub, veltoPID_pub;

  geometry_msgs::Twist velInPID;
  geometry_msgs::Pose current_pose_ref, pose_out;
  nav_msgs::Odometry odo_msg;
  double K, tol;
  double distX, distY, distZ;
};

#endif // PATH_PLANNING_HPP
