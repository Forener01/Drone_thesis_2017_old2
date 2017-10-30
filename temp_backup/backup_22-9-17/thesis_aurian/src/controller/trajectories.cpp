/*
This file transmits to the drone the trajectories to take, via ardrone_autonomy
commands.

Author: Aurian d'Avernas
Date: 2017
*/

// #include <thesis_aurian/controller/trajectories.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectories");

  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  // Creating the publisher objects
  ros::Publisher takeoff_pub =
      nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);

  // std_msgs::Empty msg;
  // takeoff_pub.publish(msg);
  // Setting values for hover mode
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;

  while (ros::ok()) {
    double time_start = (double)ros::Time::now().toSec();
    while ((double)ros::Time::now().toSec() < time_start + 5.0) {
      takeoff_pub.publish(std_msgs::Empty());

      ros::spinOnce();
      loop_rate.sleep();
    }
    //
    // vel_pub.publish(cmd);
    // ROS_INFO_STREAM("The drone is in hover mode !");
    // ros::spinOnce();
    // loop_rate.sleep();
    //
    // if ((double)ros::Time::now().toSec() > time_start + 35.0) {
    //   land_pub.publish(std_msgs::Empty());
    //   ROS_INFO_STREAM("The drone just landed !");
    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }
    ROS_INFO_STREAM("The drone just took off !");
    exit(0);
  }

  // return 0;
}
