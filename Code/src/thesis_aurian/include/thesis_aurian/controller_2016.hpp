#ifndef CONTROLLER_2016_HPP
#define CONTROLLER_2016_HPP

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

class Controller_2016 {
private:
  ros::NodeHandle nh;

  ros::Subscriber pose_sub;
  ros::Subscriber poseref_sub;
  ros::Publisher land_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher vel_pub;
  ros::Publisher toggleState_pub;
  ros::Publisher reset_pub;

  std::string pose_channel;
  std::string poseref_channel;
  std::string control_channel;
  std::string takeoff_channel;
  std::string land_channel;
  std::string toggleState_channel;
  std::string reset_channel;

  //! is true if the controller is running
  bool isControlling;

  //! Reference
  float alt_ref;
  double x_ref;
  double y_ref;
  double yaw_ref;

  double Kp_alt;
  double Ki_alt;
  double Kd_alt;
  double Kp_yaw;
  double Kp_plan;
  double Ki_plan;
  // double Kd_plan=0.0015; //This Kd is good for y
  double Kd_plan;
  // double Ki_yaw=0.0001;
  double Ki_yaw;
  double Kd_yaw;
  double integral_alt_error;
  double integral_yaw_error;
  double integral_xy_error;
  double integral_f_error;
  double integral_l_error;
  double anti_windup_yaw;

  double regu_old_time_z; // How to initialize it?
  double regu_old_time_yaw;
  double regu_old_time_xy;
  double old_delta_alt;
  double old_delta_yaw;
  double dist_old;
  double p_term_f_old;
  double p_term_l_old;
  double alt_desired_old;
  double yaw_desired_old;
  double x_desired_old;
  double y_desired_old;
  double last_vel_z_command;
  double last_vel_yaw_command;
  double last_vel_x_command;
  double last_vel_y_command;
  double old_yaw_desired;

  geometry_msgs::Pose target_pose;
  nav_msgs::Odometry current_pose;

  void reguXY(double *xvel_cmd, double *yvel_cmd, double x_mes, double y_mes,
              double x_desired, double y_desired, double yaw,
              double regu_new_time_xy);
  void reguAltitude(double *zvel_cmd, double alt_mes, double alt_desired,
                    double regu_new_time_z);
  void reguYaw(double *yawvel_cmd, double yaw_mes, double yaw_desired,
               double regu_new_time);

  void sendVelToDrone(double pitch, double roll, double yaw_vel,
                      double zvel_cmd);

  //! Callback when pose is received
  void poseCb(const nav_msgs::Odometry &odo_msg);

  //! Callback when new pose ref is received
  void poseRefCb(const geometry_msgs::Pose &poseRef);

public:
  //! Contructor.
  Controller_2016();

  //! Destructor.
  ~Controller_2016();

  void init();

  void controlLoop();
};

#endif // Controller_2016_HPP