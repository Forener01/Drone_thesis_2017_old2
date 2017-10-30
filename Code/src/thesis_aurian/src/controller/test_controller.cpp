// #include <thesis_aurian/test_controller.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

ros::Publisher vel_pub, takeoff_pub, land_pub, reset_pub, pose_pub;

void land(void) { land_pub.publish(std_msgs::Empty()); }

void takeoff(void) { takeoff_pub.publish(std_msgs::Empty()); }

void reset(void) { reset_pub.publish(std_msgs::Empty()); }

// This function sets new velocities values and publishes them.
void load_vel(double linX, double linY, double linZ, double angZ) {
  geometry_msgs::Twist cmd;

  cmd.linear.x = linX;
  cmd.linear.y = linY;
  cmd.linear.z = linZ;
  cmd.angular.z = angZ;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

void load_pose(double posX, double posY, double posZ) {
  geometry_msgs::Pose cmd_pose;

  cmd_pose.position.x = posX;
  cmd_pose.position.y = posY;
  cmd_pose.position.z = posZ;

  pose_pub.publish(cmd_pose);
}

// This function sets the hover mode.
void hover() {
  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_controller");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);
  double sleeptime = 5.0;

  vel_pub =
      nh.advertise<geometry_msgs::Twist>("cmd_PID", 1); // with TUD Controller
  // vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); // without
  // controller
  takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);
  pose_pub = nh.advertise<geometry_msgs::Pose>("poseref", 1);

  ros::Duration(8.0).sleep();

  takeoff();
  ROS_INFO_STREAM_ONCE("The drone is taking off !");
  ros::Duration(5.0).sleep();

  ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  hover();
  ros::Duration(5.0).sleep();

  /* Test for velocity control */
  // // Moving to the left #1
  // ROS_INFO_STREAM_ONCE("Test for velocity control !");
  // load_vel(0.0, 0.15, 0.0, 0.0);
  // ros::Duration(1.5).sleep();
  // // Stabilizing #1
  // hover();
  // ros::Duration(1.5).sleep();
  //
  // // Moving backward #2
  // load_vel(-0.15, 0.0, 0.0, 0.0);
  // ros::Duration(1.5).sleep();
  // // Stabilizing #2
  // hover();
  // ros::Duration(1.5).sleep();
  //
  // // Moving to the right #3
  // load_vel(0.0, -0.15, 0.0, 0.0);
  // ros::Duration(1.5).sleep();
  // // Stabilizing #3
  // hover();
  // ros::Duration(1.5).sleep();
  //
  // // Moving forward #4
  // load_vel(0.15, 0.0, 0.0, 0.0);
  // ros::Duration(1.5).sleep();
  // // Stabilizing #2
  // hover();
  // ros::Duration(1.5).sleep();

  /* Test for position control */
  ROS_INFO_STREAM_ONCE("Test for position control !");
  load_pose(0.0, -2000000.0, 1.0);
  ros::Duration(sleeptime).sleep();
  hover();
  ros::Duration(20).sleep();

  // load_pose(-2.0, 2.0, 1.0);
  // ros::Duration(sleeptime).sleep();
  // hover();
  // ros::Duration(1.5).sleep();
  //
  // load_pose(-2.0, 0.0, 1.0);
  // ros::Duration(sleeptime).sleep();
  // hover();
  // ros::Duration(1.5).sleep();
  //
  // load_pose(0.0, 0.0, 1.0);
  // ros::Duration(sleeptime).sleep();
  // hover();
  // ros::Duration(1.5).sleep();

  // Final step
  ROS_INFO_STREAM_ONCE("The drone just finished !");
  hover();
  ros::Duration(1.5).sleep();
  land();
  ROS_INFO_STREAM_ONCE("The drone just landed !");

  return 0;
}
