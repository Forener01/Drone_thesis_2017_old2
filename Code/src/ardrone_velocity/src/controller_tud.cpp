/*
This file transmits to the drone the trajectories with velocity commands,
via ardrone_autonomy commands, following a square trajectory.

Author: Aurian d'Avernas
Date: september 2017
*/

#include <ardrone_velocity/controller_tud.hpp>
#include <random>
#include <stdio.h>

Controller_TUD::Controller_TUD() {
  ros::NodeHandle nh;
  ros::param::get("~test_type", test_type);
  // ros::param::get("~the_speed", speed);

  // Subscribers
  velinPID_sub =
      nh.subscribe("cmd_PID_topic", 100, &Controller_TUD::velinPIDCb, this);

  odom_sub = nh.subscribe("ardrone/odometry", 100, &Controller_TUD::odomCb,
                          this, ros::TransportHints().tcpNoDelay());

  // Publishers
  veloutPID_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  error_pub = nh.advertise<geometry_msgs::Twist>("tud/vel_error_topic", 100);
  percent_error_pub =
      nh.advertise<geometry_msgs::Twist>("tud/vel_percent_error_topic", 100);
  /** Copied from ardrone_velocity package **/

  // Dynamic parameter reconfigure
  dynamic_reconfigure::Server<
      ardrone_velocity::dynamic_param_configConfig>::CallbackType f;
  f = boost::bind(&Controller_TUD::dynamic_reconfigureCb, this, _1, _2);
  m_server.setCallback(f);

  m_debug_pub = nh.advertise<std_msgs::Float64>("/ardrone_velocity/debug", 1);
  m_i_term_x = 0.0;
  m_i_term_y = 0.0;
}

void Controller_TUD::velinPIDCb(const geometry_msgs::Twist &cmd_vel_in) {
  m_current_command = cmd_vel_in; // Units in m/s
  // ROS_INFO("Velin PID msg has been received %f", m_current_command.linear.x);
  // ROS_INFO("Velin PID msg has been received %f", m_current_command.linear.y);
  // ROS_INFO("Velin PID msg has been received %f", m_current_command.linear.z);
  // ROS_DEBUG_THROTTLE(0.25, "cmd PID x: %f m/s", m_current_command.linear.x);
  // ROS_DEBUG_THROTTLE(0.25, "cmd PID y: %f m/s", m_current_command.linear.y);
  // ROS_DEBUG_THROTTLE(0.25, "cmd PID z: %f m/s", m_current_command.linear.z);
  ROS_DEBUG_THROTTLE(0.25, "Controller received input");
}

void Controller_TUD::odomCb(const nav_msgs::Odometry &odo_msg) {
  m_odo_msg = odo_msg;
  m_filtered_vel_x = m_filter_vel_x.filter(m_odo_msg.twist.twist.linear.x);
  m_filtered_vel_y = m_filter_vel_y.filter(m_odo_msg.twist.twist.linear.y);

  // ROS_DEBUG_THROTTLE(0.5, "cmd PID x: %f m/s", m_current_command.linear.x);
  // ROS_DEBUG_THROTTLE(0.5, "cmd PID y: %f m/s", m_current_command.linear.y);
  // ROS_DEBUG_THROTTLE(0.5, "cmd PID z: %f m/s", m_current_command.linear.z);

  velocity_control();
}

// void Controller::quad_odom_callback(const nav_msgs::Odometry &odo_msg) {
//   std_msgs::Float64 debug_msg;
//   m_odo_msg = odo_msg;
//   m_filtered_vel_x = m_filter_vel_x.filter(m_odo_msg.twist.twist.linear.x);
//   m_filtered_vel_y = m_filter_vel_y.filter(m_odo_msg.twist.twist.linear.y);
//   debug_msg.data = m_filtered_vel_x;
//   m_debug_pub.publish(debug_msg);
//
//   velocity_control();
// }

void Controller_TUD::velocity_control(void) {
  double p_term_x, d_term_x;
  double p_term_y, d_term_y;

  double error_x;
  double error_y;
  double error_z;

  double speed;

  geometry_msgs::Twist cmd_vel_out;

  // We limit the maximum reference speed of the quadcopter
  //! TODO: Change this into ROS parameters
  double max_vel = 0.6;
  m_current_command.linear.x = std::min(max_vel, m_current_command.linear.x);
  m_current_command.linear.y = std::min(max_vel, m_current_command.linear.y);

  m_current_command.linear.x = std::max(-max_vel, m_current_command.linear.x);
  m_current_command.linear.y = std::max(-max_vel, m_current_command.linear.y);

  // We are only going to change linear.x and linear.y of this command
  // The rest of the values are the same
  cmd_vel_out = m_current_command;

  // In case that we receive a special command to hover
  // if (cmd_vel_out.angular.x == 0 && cmd_vel_out.angular.y == 0 &&
  //     cmd_vel_out.angular.z == 0 && cmd_vel_out.linear.x == 0 &&
  //     cmd_vel_out.linear.y == 0 && cmd_vel_out.linear.z == 0) {
  //   set_hover();
  //   // reset iterm
  //   m_i_term_x = 0.0;
  //   m_i_term_y = 0.0;
  //   return;
  // }

  // otherwise we dont want to hover.
  cmd_vel_out.angular.x = 1;
  cmd_vel_out.angular.y = 1;

  // Control starts here
  //! TODO: Separate filter for input of the derivative term.
  //! TODO: Consider measurement and controled variables delays.

  if (m_current_command.linear.x != 0.0) {
    speed = m_current_command.linear.x;
  } else if (m_current_command.linear.y != 0.0) {
    speed = m_current_command.linear.y;
  } else {
    speed = 0.0;
  }

  // We calculate the velocity error
  error_x = m_current_command.linear.x - m_odo_msg.twist.twist.linear.x;
  error_y = m_current_command.linear.y - m_odo_msg.twist.twist.linear.y;
  error_z = m_current_command.linear.z - m_odo_msg.twist.twist.linear.z;

  // ADDED

  error_msg.linear.x = error_x;
  error_msg.linear.y = error_y;
  error_msg.linear.z = error_z;

  percent_error_msg.linear.x = error_x / speed;
  percent_error_msg.linear.y = error_y / speed;
  percent_error_msg.linear.z = error_z / speed;

  // if (m_current_command.linear.x == 0.0) {
  //   if (error_x < 0.0) {
  //     percent_error_msg.linear.x = -error_x / 0.6;
  //   } else {
  //     percent_error_msg.linear.x = error_x / 0.6;
  //   }
  //
  // } else {
  //   if ((error_x < 0.0 && m_current_command.linear.x > 0.0) ||
  //       (error_x > 0.0 && m_current_command.linear.x < 0.0)) {
  //     percent_error_msg.linear.x = -error_x / m_current_command.linear.x;
  //   } else {
  //     percent_error_msg.linear.x = error_x / m_current_command.linear.x;
  //   }
  // }
  //
  // if (m_current_command.linear.y == 0.0) {
  //   if (error_y < 0.0) {
  //     percent_error_msg.linear.y = -error_y / 0.6;
  //   } else {
  //     percent_error_msg.linear.y = error_y / 0.6;
  //   }
  //
  // } else {
  //   if ((error_y < 0.0 && m_current_command.linear.y > 0.0) ||
  //       (error_y > 0.0 && m_current_command.linear.y < 0.0)) {
  //     percent_error_msg.linear.y = -error_y / m_current_command.linear.y;
  //   } else {
  //     percent_error_msg.linear.y = error_y / m_current_command.linear.y;
  //   }
  // }
  //
  // if (m_current_command.linear.z == 0.0) {
  //   if (error_y < 0.0) {
  //     percent_error_msg.linear.z = -error_z;
  //   } else {
  //     percent_error_msg.linear.z = error_z;
  //   }
  //
  // } else {
  //   if ((error_y < 0.0 && m_current_command.linear.z > 0.0) ||
  //       (error_y > 0.0 && m_current_command.linear.z < 0.0)) {
  //     percent_error_msg.linear.z = -error_z;
  //   } else {
  //     percent_error_msg.linear.z = error_z;
  //   }
  // }

  error_pub.publish(error_msg);
  percent_error_pub.publish(percent_error_msg);

  // The proportional term is directly the error
  p_term_x = error_x;
  p_term_y = error_y;
  // p_term_x = 0;
  // p_term_y = 0;

  // For derivative and integral we need the current time and timestep (dt)
  t = ros::Time::now();
  ros::Duration dt = t - old_t;
  old_t = t;

  // Derivative term (based on velocity change instead of error change)
  // Note that we put the negative part here
  // d_term_x = -(m_odo_msg.twist.twist.linear.x - m_last_vel_x)/dt.toSec();
  // d_term_y = -(m_odo_msg.twist.twist.linear.y - m_last_vel_y)/dt.toSec();
  d_term_x = -(m_filtered_vel_x - m_last_vel_x) / dt.toSec();
  d_term_y = -(m_filtered_vel_y - m_last_vel_y) / dt.toSec();

  // std_msgs::Float64 debug_msg;
  // m_debug_pub.publish(debug_msg);

  m_last_vel_x = m_filtered_vel_x;
  m_last_vel_y = m_filtered_vel_y;

  //! You can use this version as well but it will have some discontinuities
  //! when the reference changes
  // d_term_x = (error_x - m_last_error_x)/dt.toSec();
  // d_term_y = (error_y - m_last_error_y)/dt.toSec();
  // m_last_error_x = error_x;
  // m_last_error_y = error_y;

  //! Taken from tum_autonomy package.
  //! This calculates and limits the integral term
  // m_i_term is a member of the class
  i_term_increase(m_i_term_x, error_x * dt.toSec(), 1.2);
  i_term_increase(m_i_term_y, error_y * dt.toSec(), 1.2);

  // m_i_term_x = m_i_term_x + error_x*dt.toSec();
  // m_i_term_y = m_i_term_y + error_y;//*dt.toSec();

  // Control command (PID)
  cmd_vel_out.linear.x =
      m_Kp_xy * (p_term_x + m_Ki_xy * m_i_term_x + m_Kd_xy * d_term_x);
  cmd_vel_out.linear.y =
      m_Kp_xy * (p_term_y + m_Ki_xy * m_i_term_y + m_Kd_xy * d_term_y);

  // Limit control command to min max values of ardrone SDK (-1.0, 1.0)
  cmd_vel_out.linear.x = std::min(cmd_vel_out.linear.x, 1.0);
  cmd_vel_out.linear.y = std::min(cmd_vel_out.linear.y, 1.0);

  cmd_vel_out.linear.x = std::max(cmd_vel_out.linear.x, -1.0);
  cmd_vel_out.linear.y = std::max(cmd_vel_out.linear.y, -1.0);

  // Debugging information
  double period = 0.25; // in unit of s
  // ROS_DEBUG_THROTTLE(period, "d_Time  : %f", dt.toSec());
  // ROS_DEBUG_THROTTLE(period, "VelRef_x: %f", m_current_command.linear.x);
  // ROS_DEBUG_THROTTLE(period, "VelRef_y: %f", m_current_command.linear.y);
  // // ROS_DEBUG("VelOdo : %f", m_odo_msg.twist.twist.linear.x);
  // ROS_DEBUG_THROTTLE(period, "Error_x : %f", error_x);
  // ROS_DEBUG_THROTTLE(period, "Error_y : %f", error_y);
  // ROS_DEBUG_THROTTLE(period, "cmd_vel_out_x : %f", cmd_vel_out.angular.x);
  // ROS_DEBUG_THROTTLE(period, "cmd_vel_out_y : %f", cmd_vel_out.angular.y);
  // // ROS_INFO("cmd_vel_out_z : %f", cmd_vel_out.angular.z);
  // // ROS_INFO("pterm | iterm | dterm   : %f | %f | %f", m_Kp_x*p_term_x,
  // // m_Kp_x*m_Ki_x*m_i_term_x, m_Kp_x*m_Kd_x*d_term_x);
  // ROS_DEBUG_THROTTLE(period,
  //                    "------------------------------------------------------");

  // We publish the command
  veloutPID_pub.publish(cmd_vel_out);
  ROS_DEBUG_THROTTLE(period, "Controller is publishing !");
}

void Controller_TUD::set_hover(void) {
  geometry_msgs::Twist cmd_vel_out;
  ROS_INFO_ONCE("Sending Hover command");
  cmd_vel_out.linear.x = 0;
  cmd_vel_out.linear.y = 0;
  cmd_vel_out.linear.z = 0;
  cmd_vel_out.angular.x = 0;
  cmd_vel_out.angular.y = 0;
  cmd_vel_out.angular.z = 0;

  veloutPID_pub.publish(cmd_vel_out);
}

void Controller_TUD::i_term_increase(double &i_term, double new_err,
                                     double cap) {
  if (new_err < 0 && i_term > 0)
    i_term = std::max(0.0, i_term + 2.5 * new_err);
  else if (new_err > 0 && i_term < 0)
    i_term = std::min(0.0, i_term + 2.5 * new_err);
  else
    i_term += new_err;

  if (i_term > cap)
    i_term = cap;
  if (i_term < -cap)
    i_term = -cap;
}

void Controller_TUD::dynamic_reconfigureCb(
    ardrone_velocity::dynamic_param_configConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f", config.Kp_xy, config.Ki_xy,
           config.Kd_xy);
  // Coefficients for the PID controller
  m_Kp_xy = config.Kp_xy;
  m_Ki_xy = config.Ki_xy;
  m_Kd_xy = config.Kd_xy;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_tud");
  Controller_TUD controlnode;
  ros::Rate loop_rate(2000);

  ROS_INFO_STREAM("controller_tud node started!");

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
