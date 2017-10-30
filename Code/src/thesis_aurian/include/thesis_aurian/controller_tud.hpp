#ifndef CONTROLLER_TUD_HPP
#define CONTROLLER_TUD_HPP

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <thesis_aurian/dynamic_param_configConfig.h>
#include <thesis_aurian/filtervelocity.hpp>

class Controller_TUD {

public:
  Controller_TUD();
  ros::NodeHandle nh;

  // ROS message callbacks
  void velinPIDCb(const geometry_msgs::Twist &cmd_vel_in);
  void odomCb(const nav_msgs::Odometry &odo_msg);
  void
  dynamic_reconfigureCb(ardrone_velocity::dynamic_param_configConfig &config,
                        uint32_t level);

  // PID Controller
  void velocity_control(void);

  // Taken from tum_ardrone
  void i_term_increase(double &i_term, double new_err, double cap);

  void set_hover(void);

  // Filter for quadcopter velocities
  FilterVelocity m_filter_vel_x;
  FilterVelocity m_filter_vel_y;

private:
  ros::Subscriber velinPID_sub;
  ros::Subscriber odom_sub;
  ros::Publisher veloutPID_pub;
  ros::Publisher m_debug_pub;

  // dynamic reconfigure server
  dynamic_reconfigure::Server<ardrone_velocity::dynamic_param_configConfig>
      m_server;
  geometry_msgs::Twist m_current_command;
  nav_msgs::Odometry m_odo_msg;
  ros::Time t;
  ros::Time old_t;
  double m_filtered_vel_x;
  double m_filtered_vel_y;
  double m_last_error_x;
  double m_last_error_y;
  double m_last_vel_x;
  double m_last_vel_y;
  double m_i_term_x;
  double m_i_term_y;
  // PID Coefficients
  double m_Kp_xy, m_Ki_xy, m_Kd_xy;
};

#endif // CONTROLLER_TUD_HPP
